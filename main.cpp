/***********************************************
 * project: SantaKaggle
 * 
 * Lift a LKH tour to a real solution 
 * 
 * date: 2022-12-12
 ***********************************************/


#include "mtools/mtools.hpp" 
using namespace mtools;

#include "ToolsPrograms.h"
#include "Arm.h"
#include "LKHTour.h"
#include "Vizualize.h"
#include "TreeSearch.h"
#include "SwapTour.h"
#include "distanceArm.h"
#include "Solution.h"
#include "Rectify.h"
#include "cutTime.h"
#include "handEdit.h"

MT2004_64 gen; 



thread_local Chrono ch; 

thread_local mtools::MT2004_64 genl;
    

/**
 * Return a random child. 
 **/
inline Arm trivial_heuristic(int n, Arm arm, iVec2 target, PotSon & potson, bool backtracked, TreeSearch * TS)
        {

        double w[1000];  

        double A7 = 1.0; 
        double B7 = 1.0;
        double C7 = 1.0; 

        double A6 = 1.0;
        double B6 = 1.0;
        double C6 = 1.0;

        auto t = (ch.elapsed() % 9000);  
        int i = (int)(t / 1000);
        int u7 = i / 3; 
        int u6 = i % 3; 

        if (u7 == 0)
            {
            A7 = 500;// *Unif(genl);
            C7 = 1.0 / A7;
            }
        else if (u7 ==2)
            {
            C7 = 500;// *Unif(genl);
            A7 = 1.0 / C7;
            }
        double tot = 0; 
        for (int i = 0; i < potson.size(); i++)
            {           
            const Arm a = potson[i] - arm;
            tot += (a.angle(6) == 0) ? B6 : ((a.angle(6) == 1) ? A6 : C6);
            tot += (a.angle(7) == 0) ? B7 : ((a.angle(7) == 1) ? A7 : C7);
            w[i] = tot;
            }
        for (int i = 0; i < potson.size(); i++)
            {
            w[i] /= tot;
            }
        return potson.choice(w);
        }




/**
* Main routine for lifting up a path
**/
void parallelize(const std::vector<iVec2> & tour, int nb_inst, const std::string filename)
        {
        TreeSearch * TS[256]; 
        MT2004_64 *  mtgen[256];

        for (int i = 0; i < nb_inst; i++)
            {
            mtgen[i] = new MT2004_64(Unif_32(gen)+ i*i*i);
            TS[i] = new TreeSearch(tour, *(mtgen[i]));
            TS[i]->search(trivial_heuristic);
            }

        cout.resize(50, 50, 520, 600);

        Chrono ch; 

        double best_loss = mtools::INF; 
        while(1)
            {           
            std::vector<int> ind(nb_inst, 0);
            for (int i = 0; i != ind.size(); i++)  { ind[i] = i; }
            sort(ind.begin(), ind.end(),
                [&](const int& a, const int& b) 
                {
                if (TS[a]->bestpos() > TS[b]->bestpos()) return true;
                if ((TS[a]->bestpos() == TS[b]->bestpos()) && (TS[a]->jump_loss() < TS[b]->jump_loss())) return true;
                if ((TS[a]->bestpos() == TS[b]->bestpos()) && (TS[a]->jump_loss() == TS[b]->jump_loss()) && (TS[a]->jump_steps() < TS[b]->jump_steps())) return true;

                return (TS[a]->bestpos() > TS[b]->bestpos());
                });

            int nbon = 0;
            cout.clear();
            cout << "Tour   : " << filename << "\n"; 
            cout << "length : " << tour.size() << "\n";
            cout << "score  : " << score(tour) << "\n\n";

            cout << TS[0]->hrule();
            cout << TS[0]->header();
            cout << TS[0]->hrule();
            for (int j = 0; j < nb_inst; j++)
                {
                int i = ind[j]; 
                if (!(TS[i]->solved()))
                    {
                    cout << *(TS[i]);
                    nbon++; 
                    }
                else
                    {
                    double l = TS[i]->cumulative_loss();
                    cout << "*** solved with loss : " << l << " ***\n\n\n";
                    if (l < best_loss)
                        {
                        best_loss = l; 
                        if (l == 0)
                            {
                            TS[i]->save(filename + ".lossless");
                            for (int i = 0; i < nb_inst; i++)
                                {
                                TS[i]->stopSearch();
                                }
                            return; 
                            }
                        else
                            {
                            TS[i]->save(filename + ".loss " + doubleToStringNice(((int)(l * 1000)) / 1000.0));
                            }
                        }
                    }
                }
            cout << TS[0]->hrule();
            if (nbon == 0) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }






/**
 * 
 * Main program. Take a LKH .tour file as input and output a solution (if possible)
 * 
**/
int main(int argc, char *argv[]) 
    {
    MTOOLS_SWAP_THREADS(argc,argv); // required on OSX, does nothing on Linux/Windows
    mtools::parseCommandLine(argc,argv,true); // parse the command line, interactive mode


    std::string tourname = arg("tour filename", "../LKHtours/ttr_f_7407570654169005365590.tour");

    // load the tour found with LKH 
    auto V = loadLKHTour(tourname);

    // display the tour in the plotter.     
    drawTour(V);

    // split the tour in 5 pieces (split at corners)
    std::vector<iVec2> A, B, C, D, E;
    splitTour(V, A, B, C, D, E);

     
    int nbthread = 10; // number of thread to use

    // these 3 path are trivial to lift up. 
    parallelize(B, 10, tourname + ".B");
    parallelize(C, 10, tourname + ".C");
    parallelize(D, 10, tourname + ".D");

    // these 2 are the difficult ones ! 
    parallelize(A, 10, tourname + ".A");
    parallelize(E, 10, tourname + ".E");

    // load the 5 partial solutions
    auto AA = loadSolution((tourname + ".A.lossless").c_str());
    auto BB = loadSolution((tourname + ".B.lossless").c_str());
    auto CC = loadSolution((tourname + ".C.lossless").c_str());
    auto DD = loadSolution((tourname + ".D.lossless").c_str());
    auto EE = loadSolution((tourname + ".E.lossless").c_str());

    // merge them into a single ful solution
    auto SOL = patch(AA, BB, CC, DD, EE);

    // and save it to disk. 
    saveSolution(SOL, (tourname + ".solved.csv").c_str());

    // all done !
    cout << "\n\n"; 
    cout << "*** Done. Tour successfully lifted ! ***\n\n";
    cout << "-> final score: " << mtools::doubleToStringHighPrecision(score(SOL,true)) << "\n\n";
    cout << "-> solution saved in file [" << tourname << ".solved.csv]\n\n";
    mtools::cout.getKey();	
    return 0;
    }
	
/* end of file main.cpp */

