#pragma once

#include "Arm.h"



/*
* Compute the distance between x and y aon the torus Z/lZ
* x and y must be in [0, l-1]
*/
inline int dtorus(int x, int y, int l)
    {
    if (x <= y)
        {
        const int k1 = y - x;
        const int k2 = x + l - y;
        return (k1 < k2) ? k1 : k2;
        } 
    else
        {
        const int k1 = x - y;
        const int k2 = y + l - x;
        return (k1 < k2) ? k1 : k2;
        }
    }


/**
* Compute the penalty between two arms configuration.
* Return +INF if not reachable in a single step. 
**/
inline double penaltyL1(Arm a, Arm b)
    {
    int m = 0;
    int d0 = dtorus(a._A0, b._A0, 8);
    if (d0 > 1) return mtools::INF;
    m += d0;
    int d1 = dtorus(a._A1, b._A1, 8);
    if (d1 > 1) return mtools::INF;
    m += d1;
    int d2 = dtorus(a._A2, b._A2, 16);
    if (d2 > 1) return mtools::INF;
    m += d2;
    int d3 = dtorus(a._A3, b._A3, 32);
    if (d3 > 1) return mtools::INF;
    m += d3;
    int d4 = dtorus(a._A4, b._A4, 64);
    if (d4 > 1) return mtools::INF;
    m += d4;
    int d5 = dtorus(a._A5, b._A5, 128);
    if (d5 > 1) return mtools::INF;
    m += d5;
    int d6 = dtorus(a._A6, b._A6, 256);
    if (d6 > 1) return mtools::INF;
    m += d6;
    int d7 = dtorus(a._A7, b._A7, 512);
    if (d7 > 1) return mtools::INF;
    m += d7;
    return sqrt(m);
    }



/**
* Return the loss of moving from a to b compared to the sqrt(L1) norm on the pixel.
**/
inline double lossL1(Arm a, Arm b)
    {
    const iVec2 P = a.pos() - b.pos();
    return penaltyL1(a, b) - sqrt(abs(P.X()) + abs(P.Y()));
    }



/**
* Return the cumulative loss of a path
**/
inline double lossL1(const std::vector<Arm>& V)
    {
    double s = 0;
    for (int i = 0; i < V.size() - 1; i++)
        {
        s += lossL1(V[i], V[i + 1]);
        }
    return s;
    }



/**
* Create a vector with all the losses and their positions. 
**/
inline std::vector<std::pair<int, double>> lossL1Vec(const std::vector<Arm>& V)
    {
    std::vector<std::pair<int, double>> res;
    for (int i = 0; i < V.size() - 1; i++)
        {
        double l = lossL1(V[i], V[i + 1]);
        if (l != 0)
            {
            res.push_back({ i, l });
            }
        }
    return res;
    }












/**
* Class use for computing path from 'arm' to 'arm'
**/
class ArmToArm
    {

    public:


    /**
     * Set a path from Arm a to Arm d
     **/
    void set(Arm a, Arm d)
        {
        _a = a; 
        _b = d - a; 

        for (int k = 0; k < 8; k++) { _order[k] = k; }

        std::sort(_order.begin(), _order.end(),
            [&](const int& u, const int& v)
                {
                return (_norm_unsorted(u) < _norm_unsorted(v));
                });

        _steps = _norm_sorted(7); 

        _sqrtL1 = (_norm_sorted(7) - _norm_sorted(6)) * 1 +
                  (_norm_sorted(6) - _norm_sorted(5)) * sqrt(2) +
                  (_norm_sorted(5) - _norm_sorted(4)) * sqrt(3) +
                  (_norm_sorted(4) - _norm_sorted(3)) * sqrt(4) +
                  (_norm_sorted(3) - _norm_sorted(2)) * sqrt(5) +
                  (_norm_sorted(2) - _norm_sorted(1)) * sqrt(6) +
                  (_norm_sorted(1) - _norm_sorted(0)) * sqrt(7) +
                  _norm_sorted(0) * sqrt(8);
        }

    /**
    * Return the minimum number of steps needed.
    * (when each arm can move only by one unit). 
    * 
    **/
    double steps()
        {
        return _steps;
        }


    /**
    * Return the minimum sqrt(L1) cost. 
    * (not counting image cost). 
    **/
    double sqrtL1()
        {
        return _sqrtL1;
        }



    private: 


    int _norm_unsorted(int index_arm)
        {
        return dtorus(_b.angle(index_arm), 0, 8 * _b.lenArm(index_arm));
        }


    int _norm_sorted(int index_sorted)
        {
        return _norm_unsorted(_order[index_sorted]);
        }

    Arm                 _a; 
    Arm                 _b; 
    std::array<int, 8>  _order; 

    double              _steps;
    double              _sqrtL1;
    };







class ArmToPixel
    {

    public:


        ArmToPixel(MT2004_64 & gen) : _gen(gen)
            {
            _cost = mtools::INF;
            _steps = mtools::INF;           
            _createNeighbour();
            _ch.reset();
            _busyt = 0; 
            }



        /**
        * Set a (collection of) path from a to pixel P. 
        * 
        * a = start point
        * P = end pixel
        * precision3 in [2, 8] for calculating the ball of size 3. 
        * 
        * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        * 
        * Important : here we assume that we already kknow that a and
        * P are not at distance 1 in the configuration graph (use PotSon 
        * beforev to make sure of it). 
        * 
        * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        **/
        void set(Arm a, iVec2 P, int precision2, int precision3)
            {
            auto start = std::chrono::high_resolution_clock::now();
            _set(a, P, precision2, precision3, false, Arm());
            auto elapsed = std::chrono::high_resolution_clock::now() - start;
            _busyt += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
            }



        /**
        * Arm to Arm, use it to recover a previous solution. 
        **/
        void set(Arm a, Arm target, int precision2, int precision3)
            {
            _set(a, target.pos(), precision2, precision3, true, target);
            }



        /**
        * Return the ratio of time spent inside this object. 
        **/
        double TimeUsage()
            {
            return ((double)_busyt) / (1000.0 * (_ch.elapsed()));
            }


        /**
        * Return the minimum number of steps needed
        **/
        double steps()
            {
            return _steps;
            }


        /**
        * Return the loss of the best path found
        * compared to the image penalty + L1 norm. 
        **/
        double loss()
            {
            iVec2 Q = _a.pos(); 
            auto R = Q - _P; 
            double pl1 = sqrt(abs(R.X()) + abs(R.Y()));
            double dc = distcol(Q, _P);           
            return _cost - pl1 - dc  -((_steps < mtools::INF) ? (0.0000001 * _steps) : 0);
            }


        /**
        * Return the best path found or an empty vec if noone found.
        **/
        std::vector<Arm> best_path()
            {
            std::vector<Arm> v;
            if (_cost == mtools::INF) return v; 
            v.push_back(_a1);
            v.push_back(_a2);
            if (_steps == 2) return v;
            v.push_back(_a3);
            if (_steps == 3) return v;
            v.push_back(_a4);
            return v;
            }


        /**
        * Return the end point of the path. 
        * (Can be used with 'reconstitute()' to recover the full path later. 
        **/
        Arm endPath()
            {
            MTOOLS_INSURE(_cost < mtools::INF);
            if (_steps == 2) return _a2;
            if (_steps == 3) return _a3;
            return _a4;
            }




        std::vector<Arm> expandPath(const std::vector<Arm> & path, int precision2, int precision3)
            {
            std::vector<Arm> expath;
            expath.reserve(path.size() + 100);
            expath.push_back(path[0]);
            for (size_t i = 1; i < path.size(); i++)
                {
                if (penaltyL1(path[i - 1], path[i]) == mtools::INF)
                    { // jump
                    set(path[i - 1], path[i], precision2, precision3);
                    auto P = best_path();
                    MTOOLS_INSURE(P.size() >= 2);
                    for (size_t j = 0; j < P.size(); j++) expath.push_back(P[j]);
                    }
                else
                    { // no jump
                    expath.push_back(path[i]);
                    }
                }
            return expath;
            }




    private: 


     
        void _set(Arm a, iVec2 P, int precision2, int precision3, bool use_target, Arm target)
            {
            _a = a; 
            _P = P; 
            _cost = mtools::INF;
            _steps = mtools::INF;
            int sm2 = 1000;
            int sm3 = 1000;
            int sm4 = 1000;
            if (a.centerBox(1) != P)
                { 
                // normal case: compute the extremal config a P. 
                auto _V = a.pathToReach(P);
                for (int i = 0; i < 128; i++)
                    {
                    Arm b = _V[i] - a;
                    int st = 0;
                    int sm = 0; 
                    for (int k = 0; k < 8; k++)
                        {
                        int u = dtorus(b.angle(k), 0, 8 * b.lenArm(k));;
                        if (u > st) st = u;
                        sm += u;
                        }  
                    if ((st == 2) && (sm < sm2)) sm2 = sm;
                    if ((st == 3) && (sm < sm3)) sm3 = sm;
                    if ((st == 4) && (sm < sm3)) sm4 = sm;
                    if (st < _steps) _steps = st;
                    }
                if (_steps > 4) return; 
                }            

            if (_steps == mtools::INF)
                { // special case
                _ball2(precision2, 3, use_target, target);
                if (_steps == mtools::INF) _ball3(precision3, 3, use_target, target);
                MTOOLS_INSURE(_steps <= 3);
                return; 
                }
            if (_steps == 2)
                {
                _ball2(precision2, sm2, use_target, target);
                return;
                }

            if (_steps == 3)
                {
                _ball3(precision3, sm3, use_target, target);
                return;
                }

            if (_steps == 4)
                {
                _ball4(sm4, use_target, target);
                return;
                }
            MTOOLS_ERROR("IMPOSSIBLE");

            }




        /**
        * Explore the full ball of radius 2.
        **/
        void _ball2(int maxL, int sm, bool use_target, Arm target)
            {
            std::vector<std::array<Arm, 2>> sol; 
            if (sm > (2 * maxL)) return; // there cannot be a solution
            double cost = mtools::INF;  // infinite loss
            const iVec2 Q = _a.pos(); // start pixel 
            const int end1 = std::min(maxL, sm);
            for (int i1 = 1; i1 <= end1; i1++)
                {
                const double e1 = sqrt(i1);
                if (e1 <= cost)
                    {
                    const int end2 = std::min(maxL, sm - i1);
                    for (int i2 = 1; i2 <= end2; i2++)
                        {
                        const double e2 = e1 + sqrt(i2);
                        if (e2 <= cost)
                            {
                            for (Arm n1 : _neig[i1])
                                {
                                const iVec2 P1 = (n1 + _a).pos();
                                const double e3 = e2 + distcol(Q, P1);
                                if (e3 <= cost)
                                    {
                                    for (Arm n2 : _neig[i2])
                                        {
                                        const iVec2 P2 = (n1 + n2 + _a).pos();
                                        const double e4 = e3 + distcol(P1, P2);
                                        if ((e4 <= cost) && (P2 == _P))
                                            {
                                            if ((!use_target) || (_a + n1 + n2 == target))
                                                {
                                                if (e4 < cost) sol.clear();
                                                sol.push_back({ _a + n1 , _a + n1 + n2 });
                                                cost = e4;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }  
            if (sol.size())
                {
                if (cost < _cost)
                    {
                    _steps = 2;
                    _cost = cost;
                    const int64_t i = Unif_int(0, sol.size() - 1, _gen);
                    _a1 = (sol[i])[0];
                    _a2 = (sol[i])[1];
                    }
                }
            }



        /**
        * Explore the ball of radius 3 (but use only simple moves). 
        * 
        * maxL in [2, 8]
        **/
        void _ball3(int maxL, int sm, bool use_target, Arm target)
            {
            if (sm > (3 * maxL)) return; // there cannot be a solution
            double cost = mtools::INF;  // infinite loss
            const iVec2 Q = _a.pos(); // start pixel
            std::vector<std::array<Arm, 3>> sol;
            const int end1 = std::min(maxL, sm);
            for (int i1 = 1; i1 <= end1; i1++)
                {
                const double e1 = sqrt(i1);
                if (e1 <= cost)
                    {
                    const int end2 = std::min(maxL, sm - i1);
                    for (int i2 = 1; i2 <= end2; i2++)
                        {
                        const double e2 = e1 + sqrt(i2);
                        if (e2 <= cost)
                            {
                            const int end3 = std::min(maxL, sm - i1 - i2);
                            for (int i3 = 1; i3 <= end3; i3++)
                                {
                                const double e3 = e2 + sqrt(i3);
                                if (e3 <= cost)
                                    {
                                    for (Arm n1 : _neig[i1])
                                        {
                                        const iVec2 P1 = (n1 + _a).pos();
                                        const double e4 = e3 + distcol(Q, P1);
                                        if (e4 <= cost)
                                            {
                                            for (Arm n2 : _neig[i2])
                                                {
                                                const iVec2 P2 = (n1 + n2 + _a).pos();
                                                const double e5 = e4 + distcol(P1, P2);
                                                if (e5 <= cost)
                                                    {
                                                    for (Arm n3 : _neig[i3])
                                                        {
                                                        const iVec2 P3 = (n1 + n2 + n3 + _a).pos();
                                                        const double e6 = e5 + distcol(P2, P3);
                                                        if ((e6 <= cost) && (P3 == _P))
                                                            {
                                                            if ((!use_target) || (_a + n1 + n2 + n3 == target))
                                                                {
                                                                if (e6 < cost) sol.clear();
                                                                sol.push_back({ _a + n1 , _a + n1 + n2, _a + n1 + n2 + n3 });
                                                                cost = e6;
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            if (sol.size() > 0)
                {
                if (cost < _cost)
                    {
                    _steps = 3;
                    _cost = cost;
                    const int64_t i = Unif_int(0, sol.size() - 1, _gen);
                    _a1 = (sol[i])[0];
                    _a2 = (sol[i])[1];
                    _a3 = (sol[i])[2];
                    }
                }
            }



        /**
        * Explore the ball of radius 4 (but only for move of form  2 1/2 1/2 1
        **/
        void _ball4(int sm, bool use_target, Arm target)
            {
            switch (sm)
                {
                case 5: { _ball4_5(use_target, target); break; }
                case 6: { _ball4_6(use_target, target); break; }
                case 7: { _ball4_7(use_target, target); break; }
                }
            }



        void _ball4_5(bool use_target, Arm target)
            {
            double cost = mtools::INF;  // infinite loss
            const iVec2 Q = _a.pos(); // start pixel
            std::vector<std::array<Arm, 4>> sol;
            const double fixed_cost = (sqrt(2) + 3);
            for (Arm n1 : _neig[2])
                {
                const iVec2 P1 = (n1 + _a).pos();
                const double e1 = distcol(Q, P1) + fixed_cost;;
                if (e1 <= cost)
                    {
                    for (Arm n2 : _neig[1])
                        {
                        const iVec2 P2 = (n1 + n2 + _a).pos();
                        const double e2 = e1 + distcol(P1, P2);
                        if (e2 <= cost)
                            {
                            for (Arm n3 : _neig[1])
                                {
                                const iVec2 P3 = (n1 + n2 + n3 + _a).pos();
                                const double e3 = e2 + distcol(P2, P3);
                                if (e3 <= cost)
                                    {
                                    for (Arm n4 : _neig[1])
                                        {
                                        const iVec2 P4 = (n1 + n2 + n3 + n4 + _a).pos();
                                        const double e4 = e3 + distcol(P3, P4);
                                        if ((e4 <= cost) && (P4 == _P))
                                            {
                                            if ((!use_target) || (n1 + n2 + n3 + n4 + _a == target))
                                                {
                                                if (e4 < cost) sol.clear();
                                                sol.push_back({ _a + n1 , _a + n1 + n2, _a + n1 + n2 + n3, _a + n1 + n2 + n3 + n4 });
                                                cost = e4;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            if (sol.size() > 0)
                {
                if (cost < _cost)
                    {
                    _steps = 4;
                    _cost = cost;
                    const int64_t i = Unif_int(0, sol.size() - 1, _gen);
                    _a1 = (sol[i])[0];
                    _a2 = (sol[i])[1];
                    _a3 = (sol[i])[2];
                    _a4 = (sol[i])[3];
                    }
                }
            }



        /**
        * Explore the ball of radius 4 (but use only move of size 1).
        **/
        void _ball4_6(bool use_target, Arm target)
            {
            double cost = mtools::INF;  // infinite loss
            const iVec2 Q = _a.pos(); // start pixel
            std::vector<std::array<Arm, 4>> sol;
            const double fixed_cost = (2 * sqrt(2) + 2);
            for (Arm n1 : _neig[2])
                {
                const iVec2 P1 = (n1 + _a).pos();
                const double e1 = distcol(Q, P1) + fixed_cost;;
                if (e1 <= cost)
                    {
                    for (Arm n2 : _neig[2])
                        {
                        const iVec2 P2 = (n1 + n2 + _a).pos();
                        const double e2 = e1 + distcol(P1, P2);
                        if (e2 <= cost)
                            {
                            for (Arm n3 : _neig[1])
                                {
                                const iVec2 P3 = (n1 + n2 + n3 + _a).pos();
                                const double e3 = e2 + distcol(P2, P3);
                                if (e3 <= cost)
                                    {
                                    for (Arm n4 : _neig[1])
                                        {
                                        const iVec2 P4 = (n1 + n2 + n3 + n4 + _a).pos();
                                        const double e4 = e3 + distcol(P3, P4);
                                        if ((e4 <= cost) && (P4 == _P))
                                            {
                                            if ((!use_target) || (n1 + n2 + n3 + n4 + _a == target))
                                                {
                                                if (e4 < cost) sol.clear();
                                                sol.push_back({ _a + n1 , _a + n1 + n2, _a + n1 + n2 + n3, _a + n1 + n2 + n3 + n4 });
                                                cost = e4;
                                                }
                                            }
                                        }
                                    }                            
                                }
                            }
                        }
                    }
                }
            if (sol.size() > 0)
                {        
                if (cost < _cost)
                    {
                    _steps = 4;
                    _cost = cost;
                    const int64_t i = Unif_int(0, sol.size() - 1, _gen);
                    _a1 = (sol[i])[0];
                    _a2 = (sol[i])[1];
                    _a3 = (sol[i])[2];
                    _a4 = (sol[i])[3];
                    }
                }
            }



        void _ball4_7(bool use_target, Arm target)
            {
            double cost = mtools::INF;  // infinite loss
            const iVec2 Q = _a.pos(); // start pixel
            std::vector<std::array<Arm, 4>> sol;
            const double fixed_cost = (3 * sqrt(2) + 1);
            for (Arm n1 : _neig[2])
                {
                const iVec2 P1 = (n1 + _a).pos();
                const double e1 = distcol(Q, P1) + fixed_cost;
                if (e1 <= cost)
                    {
                    for (Arm n2 : _neig[2])
                        {
                        const iVec2 P2 = (n1 + n2 + _a).pos();
                        const double e2 = e1 + distcol(P1, P2);
                        if (e2 <= cost)
                            {
                            for (Arm n3 : _neig[2])
                                {
                                const iVec2 P3 = (n1 + n2 + n3 + _a).pos();
                                const double e3 = e2 + distcol(P2, P3);
                                if (e3 <= cost)
                                    {
                                    for (Arm n4 : _neig[1])
                                        {                                     
                                        const iVec2 P4 = (n1 + n2 + n3 + n4 + _a).pos();
                                        const double e4 = e3 + distcol(P3, P4);
                                        if ((e4 <= cost) && (P4 == _P))
                                            {
                                            if ((!use_target) || (n1 + n2 + n3 + n4 + _a == target))
                                                {
                                                if (e4 < cost) sol.clear();
                                                sol.push_back({ _a + n1 , _a + n1 + n2, _a + n1 + n2 + n3, _a + n1 + n2 + n3 + n4 });
                                                cost = e4;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            if (sol.size() > 0)
                {
                if (cost < _cost)
                    {
                    _steps = 4;
                    _cost = cost;
                    const int64_t i = Unif_int(0, sol.size() - 1, _gen);
                    _a1 = (sol[i])[0];
                    _a2 = (sol[i])[1];
                    _a3 = (sol[i])[2];
                    _a4 = (sol[i])[3];
                    }
                }
            }





        Arm     _a, _a1, _a2, _a3, _a4; 
        iVec2   _P; 
        double  _steps;
        double  _cost;

        MT2004_64& _gen;


        // number of neighours depending on the number of non-zero angles. 
        //  1  2   3    4    5    6    7   8
        // 16 112 448 1120 1792 1792 1024 256
        void _createNeighbour()
            {
            for (int i = 0; i < 9; i++) _neig->clear();
            for (int u = 0; u < 6561; u++)
                {
                int nb = 0;
                int i = u;
                int A7 = i % 3; i /= 3; if (A7 == 2) A7 = -1; if (A7 != 0) nb++;
                int A6 = i % 3; i /= 3; if (A6 == 2) A6 = -1; if (A6 != 0) nb++;
                int A5 = i % 3; i /= 3; if (A5 == 2) A5 = -1; if (A5 != 0) nb++;
                int A4 = i % 3; i /= 3; if (A4 == 2) A4 = -1; if (A4 != 0) nb++;
                int A3 = i % 3; i /= 3; if (A3 == 2) A3 = -1; if (A3 != 0) nb++;
                int A2 = i % 3; i /= 3; if (A2 == 2) A2 = -1; if (A2 != 0) nb++;
                int A1 = i % 3; i /= 3; if (A1 == 2) A1 = -1; if (A1 != 0) nb++;
                int A0 = i % 3; i /= 3; if (A0 == 2) A0 = -1; if (A0 != 0) nb++;
                _neig[nb].push_back(Arm(A7, A6, A5, A4, A3, A2, A1, A0));
                }

            _neigB4.clear();
            _neigB4.reserve(_neig[1].size() + _neig[2].size());
            for (auto a : _neig[1]) _neigB4.push_back(a);
            for (auto a : _neig[2]) _neigB4.push_back(a);
            }

        std::vector<Arm> _neig[9];
        std::vector<Arm> _neigB4;

        Chrono _ch; 
        uint64_t _busyt;

    };







/** end of file */