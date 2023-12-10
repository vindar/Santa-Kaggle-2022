#pragma once



#include "mtools/mtools.hpp" 
using namespace mtools;
#include "Arm.h"
#include "distanceArm.h"
#include "PotSon.h"
#include "Rectify.h"



/**
* Exception structure. 
**/
struct ExcRange
    {
    double maxCumLoss; // maximum authorized cumulative loss after exiting the range (ie when pos > max_pos)

    int min_pos;        // beginning of the range
    int pos_max;        // end of the range. This is where the jump (may) happen. 

    double prob_jump_min; // lower bound on prob. to accept jump at pos_max
    double prob_jump_max; // upper bound on prob. to accept jump at pos_max

    double prob_detour_min; // lower bound prob. to look for a detour while in [min_pos, pos_max]
    double prob_detour_max; // upper bound prob. to look for a detour while in [min_pos, pos_max]
    int detour_max; // when looking for detour, maximum number of additiona larm movements (between 1 and 3). 
    };





class TreeSearch
    {



    public: 


        /**
         *  Ctor
         **/        
        TreeSearch(const std::vector<iVec2>& tour, MT2004_64 & gen) : 
            _nbsteps(0), _th(nullptr), _tour(tour), _gen(gen), _potson(gen), _ison(false), _request_stop(false), _ispaused(false), _request_pause(0), _G(0.5), _a2p(gen)
            {            

            // tunneling
            setTunnelingProbability();

            // temperature, annealing
            setTemperature();

            // exception 
            _cumloss.reserve(100);
            _cumloss.push_back({ -1, 0.0 });
            _exctab.reserve(100);
            setExcPeriod();

            // data            
            _best.reserve(tour.size() + 10);
            _current.reserve(tour.size() + 10);            
            Arm A(tour[0]); // must be the origin or a corner. 
            _best.push_back(A);
            _current.push_back(A);

            _nb_visit_at_best = 0; 
            _min_steps = mtools::INF;
            _min_loss = mtools::INF;
            _cum_loss_at_best = 0; 

            searchPrecision();
            }


        /**
        * Dtor
        **/
        ~TreeSearch()
            {
            stopSearch(); 
            if (_th != nullptr)
                { // delete previous thread object if needed. 
                _th->join();
                delete _th;
                }
            }


        /**
        * Set how much of the Ball of size 2 and 3 we explore.
        * precision = allow jump with up too 'precision' arm movements simultaneously.
        **/
        void searchPrecision(int precision2 = 3, int precision3 = 2)
            {
            if (precision2 < 1) precision2 = 1;
            if (precision2 > 8) precision2 = 8;
            _precision2 = precision2;

            if (precision3 < 1) precision3 = 1;
            if (precision3 > 8) precision3 = 8;
            _precision3 = precision3;
            }


        void setTunnelingProbability(double tunneling_prob = 0.000001)
            {
            _tunnel_prob = tunneling_prob;
            }


        /**
         * Set the speed and amplitude at which the probability to branch oscillates. 
         */
        void setTemperature(double min_branch_prob = 0.0035, double max_branch_prob = 0.005, int period_sec = 30)
            {              
            _anneal_period = 1000 * period_sec;
            _min_branch_prob = min_branch_prob;
            _max_branch_prob = max_branch_prob;
            }


        /**
        * Set the period of prob. oscillation for exceptions. 
        **/
        void setExcPeriod(int period_sec = 7)
            {
            _exc_period = 1000 * period_sec;;
            }



        /**
        * Set the exception we are currently working on
        **/
        void pushException( double maxCumLoss,
                            int min_pos, int pos_max,
                            double prob_jump_min = 0.001, double prob_jump_max = 0.1,
                            double prob_detour_min = 0.0001, double prob_detour_max = 0.001, int detour_max = 1)
            {
            bool ip = isPaused();
            pause(true);
            ExcRange exc;
            exc.maxCumLoss = maxCumLoss;
            exc.min_pos = min_pos;
            exc.pos_max = pos_max;
            exc.prob_jump_min = prob_jump_min;
            exc.prob_jump_max = prob_jump_max;
            exc.prob_detour_min = prob_detour_min;
            exc.prob_detour_max = prob_detour_max;
            exc.detour_max = detour_max;
            _exctab.push_back(exc);
            pause(ip);
            }

       



        /**
        * Reset the search at a fraction of the best tour.
        **/
        void resetAtBestPos(int pos)
            {
            if (pos < 0) pos = 0;
            if (pos > (int)_best.size()) pos = (int)_best.size();

            bool ip = isPaused();
            pause(true);

            _current = _best; 
            _current.resize(pos);

            pause(ip);
            }


        /**
        * Reset the search at a positon in the best tour
        **/
        void resetAtBestRatio(float ratio = 1.0)
            {
            int npos = (int)(_best.size() * ratio);
            resetAtBestPos(npos);
            }



        /**
         * Start the path search thread. 
         *
         * @param   fun         The heuristic function to choose the son. 
         */
        template<typename HEURISTIC> void search(HEURISTIC fun)
            {
            MTOOLS_INSURE(!isSearchOn());
            if (_th != nullptr)
                { // delete previous thread object if needed. 
                _th->join(); 
                delete _th; 
                }
            _th = new std::thread(&TreeSearch::_threadproc<HEURISTIC>, this, fun);
            while ((bool)_ison == false)
                {
                std::this_thread::yield();
                }
            }


        /**
        * Stop the search and return when it has ended.
        *
        * This stops the thread.
        **/
        void stopSearch()
            {
            _request_stop = true;
            while ((bool)_ison)
                {
                std::this_thread::yield();
                }
            _request_stop = false;
            }


        /**
        * Query if the search is currently ongoing
        **/
        bool isSearchOn()
            {
            return (bool)_ison;
            }


        /**
        * Pause / restart the search thread. 
        * 
        * The method wait for the thread to pause/restart before returning. 
        **/
        void pause(bool status)
            {
            if (!isSearchOn()) return; // nothing to do
            if (isPaused() == status) return; // nothing to do. 
            _request_pause = ((status) ? -1 : 1); // signal thread to pause / resume
            while (isPaused() != status)
                {
                std::this_thread::yield();
                }
            _request_pause = 0; 
            }


        /**
        * Query if the thread is currently paused. 
        **/
        bool isPaused()
            {
            return (bool)_ispaused;
            }



        /**
        * Query if we have a full solution. 
        **/
        bool solved() const
            {
            return (_best.size() == _tour.size());
            }




        /**
        * Print formattted info about the search (one line). 
        **/
        std::string toString()
            {
            double jl = jump_loss();
            jl = (jl == mtools::INF) ? jl : (((int)(jl * 1000)) / 1000.0);
            double cl = cumulative_loss();
            cl = (cl == mtools::INF) ? cl : (((int)(cl * 1000)) / 1000.0);
            mtools::ostringstream oss; 
            oss << justify_left(mtools::toString(nbsteps()), 11) << "| "
                << justify_right(mtools::toString(pos()), 5) << " / "
                << justify_left(mtools::toString(bestpos()), 5) << " | "
                << justify_right(mtools::doubleToStringNice(jump_steps()), 5) << " | "
                << justify_right(mtools::doubleToStringNice(jl), 6) << " | "
                << justify_right(mtools::toString(jump_setsize()), 7) << " | "
                << justify_right(mtools::doubleToStringNice(cl), 6) << "\n";
            return oss.toString(); 
            }

        /**
        * Header for the toString function
        **/
        std::string header()
            {
            return "nb steps   |   pos  maxpos | steps |  loss  | setsize | cumloss\n";
            }


        std::string hrule()
            {
            return "                                                               \n";
            }



        /**
        * Save the best (extended) path into a file in csv format. 
        **/
        std::string save(std::string filename)
            {
            bool ip = isPaused();
            pause(true);
            //if (add_random_number) filename += std::string(".") + mtools::toString((int)(10000000 * Unif(_gen)));
            LogFile f(filename, false, false, false);
            auto V = bestPath();           
            for (int i = 0; i < V.size(); i++)
                {
                f << V[i].str();
                }
            pause(ip);
            return filename;
            }


        void loadPartial(const std::vector<Arm>& Varm)
            {
            MTOOLS_INSURE(Varm.size() > 0); 
            bool ip = isPaused();
            pause(true);
            _best.resize(Varm.size());
            _current.resize(Varm.size());
            for (int i = 0; i < Varm.size(); i++)
                {
                _best[i] = Varm[i];
                _current[i] = Varm[i];
                }
            pause(ip);
            return;
            }


        void loadPartial(const std::string & filename)
            {
            auto V = loadSolution(filename.c_str()); 
            loadPartial(V); 
            }



        /*******************************************************************************************
        *
        * Statistics
        * 
        ********************************************************************************************/



        /**
        * Return the best (partial) solution.
        * 
        * The returned tour is extended i.e. it may  be longer than the orignal one because
        * of the jumps
        **/
        const std::vector<Arm> bestPath()
            {
            bool ip = isPaused();
            pause(true);
            auto V = _a2p.expandPath(_best, _precision2, _precision3);            
            pause(ip);
            return V;
            }


        /**
        * Return the current path
        *
        * The returned tour is extended i.e. it may  be longer than the orignal one because
        * of the jumps
        **/
        const std::vector<Arm> currentPath()
            {
            bool ip = isPaused();
            pause(true);
            auto V = _a2p.expandPath(_current, _precision2, _precision3);
            pause(ip);
            return V;
            }


        /**
        * Current position being studied
        **/
        int pos()
            {
            return (int)_current.size() - 1;
            }

        /**
        * Best position yet
        **/
        int bestpos()
            {
            return (int)_best.size() - 1;
            }

        /**
        * Return the current branching probability
        **/
        double branch_probability()
            {
            return _branch_prob;
            }


        /**
        * Return the total number of steps performed by the search. 
        **/
        int64 nbsteps()
            {
            return (int64)_nbsteps;
            }


        /**
        * minimum jump size found at the maximum distance
        **/
        double jump_steps()
            {
            return _min_steps;
            }

        /**
        * Miminum loss encountered at the current maximum
        **/
        double jump_loss()
            {
            return _min_loss; 
            }


        /**
        * Number of distinct arms found at maximum index reached. 
        **/
        int jump_setsize()
            {
            return (int)_bestset.size();
            }


        /**
        * Cumulative loss of the path w.r.t. the L1 norm on the pixels coolors.
        **/
        double cumulative_loss()
            {
            return _cum_loss_at_best;
            }











            

    private:





        /** Thread working method */
        template<typename HEURISTIC> void _threadproc(HEURISTIC fun)
            {
            _ison = true;
            _work(fun);
            _ison = false;
            _ispaused = false; 
            }


        /**
        * Main search method. 
        **/
        template<typename HEURISTIC> void _work(HEURISTIC fun)
            {
            Arm a;
            bool backtracked = false;
            _nbsteps = 0; 
            _updateTemperature();
            _updateExcTime();
            MTOOLS_INSURE(_current.size() > 0);
            int n = (int)_current.size() - 1;  // current position
            const int N = (int)_tour.size() - 1; // end position 
            while (n < N)
                {  
                //
                // check for pause / resume / stop. 
                // 
                if (((++_nbsteps) & 1023) == 0)
                    { // check for pause/stop action
                    if ((bool)(_request_stop)) return;
                    if ((int)(_request_pause) == -1)
                        { // pausing the thread. 
                        _ispaused = true;
                        while ((int)(_request_pause) != 1)
                            {
                            if ((bool)(_request_stop)) return;
                            std::this_thread::yield();
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            }
                        _ispaused = false;
                        MTOOLS_INSURE(_current.size() > 0);
                        int n = (int)_current.size() - 1;  // update current position if it has changed
                        backtracked = false;
                        }
                    _updateTemperature();
                    _updateExcTime();
                    }

                //
                // Enumerate the direct sons 
                // 
                const Arm arm = _current[n];        // arm at the current position
                const iVec2 target = _tour[n+1];    // target pixel
                _potson.clear();                    // 
                _potson.add(arm, target, 0);        // list all direct sons (i.e. without loss).


                if (_potson.size() == 0)
                    {

                    
                  //  if (n == _best.size() - 1)
                        { // at the maximum, try rectifying

                     //   cout << "------- ";
                        const int rec = rectify(_current, target);
                        if (rec > 0)
                            {
                         //   cout << "REC " << rec << "\n";
                            continue; 
                            }
                            
                        }

                    //
                    // Tunneling 
                    //
                                       
                    if (Unif(_gen) < _tunnel_prob) // * (_best.size() - _current.size()))
                        {
                        int L = (int)_best.size() - 1;
                        double a = Unif(_gen);
                        if (a < 0.2)
                            { // unif
                            L = (int)Unif_int(((int)_current.size() * 3)/ 4, L, _gen);
                            }
                        else if (a < 0.7)
                            {
                            GeometricLaw G(_branch_prob/4);
                            L -= (int)G(_gen);
                            }
                        if ((L <= 0)||(L > (int)_best.size())) L = (int)_best.size();
                                               
                        _current.resize(L);
                        for (int i = 0; i < L; i++)
                            {
                            _current[i] = _best[i];
                            }
                        n = (int)_current.size() - 1;
                        continue;
                        }
                        
                    //
                    // Cul de sac !
                    // 
                    // Check if we allow an exception jump                   
                    for (int i = ((int)_exctab.size()) - 1; i >= 0; i--)
                        {
                        auto& e = _exctab[i];
                        if (n > e.pos_max) break; // we are after all remaining exceptions in the array. 
                        if (e.pos_max == n)
                            { // Yes, we may try a jump.      


                            const double p = e.prob_jump_max * _exc_time + (1 - _exc_time) * e.prob_jump_min; // proba. to jump.  h
                            if (Unif(_gen) < p)
                                { // ok, we may try a jump


                                _a2p.set(arm, target, _precision2, _precision3); // check for possible jump. 
                                const double nloss = _a2p.loss() + _cumloss.back().second;                                
                                if (nloss <= e.maxCumLoss)
                                    { // ok, we perform the jump !
                                    _cumloss.push_back({ n, nloss });
                                    a = _a2p.endPath();
                                    goto go_further2; 
                                    }                               
                                }
                            }
                        }


                    //
                    // No exception used so we must go back. 
                    // 
                    if (n == _best.size() - 1)
                        { // we were at the maximum, collect stats
                        auto pp = _bestset.insert(arm);
                        if (pp.second)
                            { // new arm never seen before: study it...
                            _a2p.set(arm, target, _precision2, _precision3); // check ball of radius 2 and 3
                            const double st = _a2p.steps();
                            const double sl = _a2p.loss();
                            bool improved = ((sl < _min_loss) || ((_min_loss == mtools::INF) && (st < _min_steps)));
                            if (st < _min_steps) { _min_steps = st; }
                            if (sl < _min_loss) { _min_loss = sl; }
                            _nb_visit_at_best++;
                            if (improved)
                                {
                                // save the best tour
                                _cum_loss_at_best = _cumloss.back().second;
                                int i = n;
                                while (_best[i] != _current[i])
                                    {
                                    _best[i] = _current[i];
                                    i--;
                                    }
                                // clear stats
                                _bestset.clear();
                                _bestset.insert(arm);
                                }
                            }
                        }
                    // We go back
                    int b = (int)_G(_gen);
                    n -= b;
                    if (n < 0) n = 0;
                    _current.resize(n + 1);

                    int i = (int)_cumloss.size() - 1;
                    while (_cumloss[i].first >= n) i--; // sentinel at -1 prevent overflow
                    _cumloss.resize(i + 1);

                    backtracked = true;
                    continue;
                    }

                // 
                // there are sons. Let us see we add a detour...
                //                 
                for (int i = ((int)_exctab.size()) -1; i >= 0; i--)
                    {
                    auto& e = _exctab[i];
                    if (n >= e.pos_max) goto go_further;  // we are after all remaining exceptions in the array. 
                    if (e.min_pos <= n)
                        { // detour !
                        const double p = e.prob_detour_max * _exc_time_detour + (1 - _exc_time_detour) * e.prob_detour_min; // proba. to add detour sons
                        double delta = e.maxCumLoss - _cumloss.back().second; // how much we can afford to loose. 
                        for (int d = 1; d <= e.detour_max; d++)
                            {
                            const double pen = _potson.penaltyL1(d);
                            if ((pen <= delta) && (Unif(_gen) < p))
                                {
                                _potson.clear();  // remove direct sons
                                _potson.add(arm, target, d); // add the indirect ones.
                                if (_potson.size() == 0)
                                    { // can happen in some locking situations.. 
                                    _potson.add(arm, target, 0); // add back...
                                    continue;
                                    }
                                _cumloss.push_back({ n, _cumloss.back().second + pen }); // register the loss. 
                                goto go_further; 
                                }
                            }
                        }
                    }
                    
                //
                // We move one step further. 
                // 
                
            go_further: 

                a = fun(n, arm, target, _potson, backtracked, this); // pick the next arm. 

            go_further2:

                backtracked = false;
                _current.push_back(a); 
                n++;
                if (n >= _best.size())
                    { 
                    // new strict maximum ! here n == _best.size()
                    _best.push_back(a); 
                    int i = n - 1; 
                    // save the best path
                    _cum_loss_at_best = _cumloss.back().second;
                    while (_best[i] != _current[i])
                        {
                        _best[i] = _current[i];
                        i--; 
                        }
                    // clear stats
                    _nb_visit_at_best = 0;
                    _min_steps = mtools::INF;
                    _min_loss = mtools::INF;
                    _bestset.clear();
                    }
                }
            //
            // SOLVED !!!!!!!!!
            // WE can exit the thread. 
            // We must reconstittute the full path.           
            }





        
        void _updateTemperature()
            {
            _branch_prob = _updateTimeVal(_min_branch_prob, _max_branch_prob, _anneal_period, _ch_anneal, false);
            _G.setParam(_branch_prob);
            }


        void _updateExcTime()
            {
            _exc_time = _updateTimeVal(0, 1, _exc_period, _ch_exc, false);
            _exc_time_detour = _updateTimeVal(0, 1, _exc_period, _ch_exc, true);
            }


        double _ramp(double x) const 
            {
            if ((x < 0.25) || (x > 0.75)) return 0.0;
            if (x < 0.5) return 4 * (x - 0.25);
            return 4 * (0.75 - x);
            }


        double _updateTimeVal(double minval, double maxval, uint64_t period, mtools::Chrono& ch, bool reverse)
            {
            uint64_t el = ch.elapsed();
            if (el > period)
                {
                el = period;
                ch.reset();
                }
            if (reverse)
                {
                el = period - el; 
                }
            const double x = _ramp(((double)el) / period);
            return maxval * x + minval * (1 - x); 
            }



        std::atomic<int64_t> _nbsteps;  // number of steps in the search

        std::thread* _th;           // the thread object

        std::vector<iVec2>  _tour;  // the tour to rectify. 

        MT2004_64& _gen;            // RNG
        PotSon _potson;             // object to list potential sons. 

        std::atomic<bool> _ison;     // is the thread currently working. 
        std::atomic<bool> _request_stop; // true if the thread is requested to stop 

        std::atomic<bool> _ispaused;        // true is the thread is currently paused. 
        std::atomic<int> _request_pause;    // -1 = request pause, +1 request resume, 0 = nothing

        
        Chrono _ch_anneal;          // chronometer for annealing. 
        uint64 _anneal_period;      // period in milliseconds
        double _min_branch_prob;    // min branch probability
        double _max_branch_prob;    // max branch probability
        double _branch_prob;        // current branch probability
        GeometricLaw _G;            // geometric with parameter _branch_prob;


        std::vector<std::pair<int, double>> _cumloss; // array of cumulative loss. The position is that of the jump/detour (the loss start after that).
        std::vector<ExcRange> _exctab; // array of exceptions. 
        Chrono _ch_exc;             // chronometer for exceptions
        uint64 _exc_period;         // period for exceptions
        double _exc_time;           // scaling factor (depending on time) for exc. jump.
        double _exc_time_detour;    // scaling factor (depending on time) for exc. detour. 

        double _tunnel_prob;         // probability of tunneling

        std::vector<Arm>    _best;          // best solution 
        std::vector<Arm>    _current;       // current solution
        
        int64 _nb_visit_at_best; // number of visit at best index. 
        std::set<Arm, compareArm> _bestset;    // set of arms visited at current maximum
        double _min_steps;  // minimum number of step found to cross the current maximum. 
        double _min_loss; // minimum loss found to cross the current maximum. 
        double _cum_loss_at_best; // cumulative loss of the best path


        ArmToPixel _a2p; // compute short path outside of tour
        int _precision2;  // how much of the ball of size 2 we explore
        int _precision3;  // how much of the ball of size 3 we explore
    };







/** end of file */

    