
#include <mtools/mtools.hpp>
using namespace mtools;

#include "Solution.h"

#include "SantaImage.h"
#include "Arm.h"
#include "distanceArm.h"
#include "LKHtour.h"
#include "Vizualize.h"



double score(const std::vector<Arm>& arm_tour, bool strict)
    {
    double tot = 0;
    for (int i = 1; i < arm_tour.size(); i++)
        {
        double di = penaltyL1(arm_tour[i - 1], arm_tour[i]);
        if (strict)
            {
            if (di == mtools::INF)
                {
                MTOOLS_ERROR("Tour contains a forbidden moves.");
                }
            }
        tot += di + distcol(arm_tour[i - 1].pos(), arm_tour[i].pos());
        }

    if (strict)
        {
        if (arm_tour[0].pos() != iVec2(0, 0))
            {
            MTOOLS_ERROR("tour does not start at {0,0}");
            }
        if (arm_tour.back().pos() != iVec2(0, 0))
            {
            MTOOLS_ERROR("tour does not end at {0,0}");
            }

        std::set<iVec2> ss;

        for (auto a : arm_tour)
            {
            iVec2 Q = a.pos();
            ss.insert(Q);
            if ((Q.X() < -128) || (Q.X() > 128))
                {
                MTOOLS_ERROR("X value out of range [-128,128]");
                }
            if ((Q.Y() < -128) || (Q.Y() > 128))
                {
                MTOOLS_ERROR("Y value out of range [-128,128]");
                }
            }
        if (ss.size() != 257 * 257)
            {

            for (int y = -128; y <= 128; y++)
                {
                for (int x = -128; x <= 128; x++)
                    {
                    if (ss.find(iVec2(x, y)) == ss.end())
                        {
                        cout << "- missing point " << iVec2(x, y) << "\n";
                        }
                    }
                }
            { 
            observe(arm_tour);
            while(1)                
                { 
                std::this_thread::yield();
                }
            }
            MTOOLS_ERROR("Tour does not visit all points !");

            }
        }
    return tot;
    }



std::vector<Arm> loadSolution(const char* filename)
    {
    auto S = loadStringFromFile(filename);
    if (S.size() == 0)
        {
        MTOOLS_ERROR(std::string("loadSolution(): ERROR, CANNOT LOAD: ") + filename);
        }
    auto U = tokenize(S, "", "\n");
    std::vector<Arm> sol;
    sol.reserve(U.size() + 1);
    for (int i = 0; i < U.size(); i++)
        {
        if ((i == 0) && (U[i] == std::string("configuration"))) continue;
        Arm a;
        if (a.parse(U[i]) == false)
            {
            MTOOLS_ERROR(std::string("loadSolution(): ERROR, INVALID ARM FOR FILE: ") + filename);
            }
        sol.push_back(a);
        }
    return sol;
    }



void saveSolution(const std::vector<Arm>& arm_tour, const char* filename)
    {
    LogFile f(filename, false, false);
    f << "configuration\n";
    for (int i = 0; i < arm_tour.size(); i++)
        {
        f << arm_tour[i].str();
        }
    return;
    }



std::vector<Arm> patch(const std::vector<Arm>& A, const  std::vector<Arm>& B, const  std::vector<Arm>& C, const  std::vector<Arm>& D, const  std::vector<Arm>& E)
    {
    int U[5] = { 0,0,0,0,0 };
    std::vector<Arm> T[5];
    T[0] = A;
    T[1] = B;
    T[2] = C;
    T[3] = D;
    T[4] = E;

    // find the head
    std::vector<Arm> S;
    S.reserve(T[0].size() + T[1].size() + T[2].size() + T[3].size() + T[4].size());

    for (int i = 0; i < 5; i++)
        {
        if ((T[i]).front().pos() == iVec2(0, 0)) { S = T[i]; U[i] = 1; break; }
        if ((T[i]).back().pos() == iVec2(0, 0)) { S = getReversed(T[i]);  U[i] = 1; break; }
        }
    MTOOLS_INSURE(S.size() > 0);

    for (int n = 0; n < 4; n++)
        {
        bool e = false;
        for (int k = 0; k < 5; k++)
            {
            if (U[k] == 0)
                {
                if (T[k].front().pos() == S.back().pos())
                    {
                    for (int i = 1; i < T[k].size(); i++) { S.push_back((T[k])[i]); }
                    U[k] = 1;
                    e = true;
                    break;
                    } else if (T[k].back().pos() == S.back().pos())
                        {
                        T[k] = getReversed(T[k]);
                        for (int i = 1; i < T[k].size(); i++) { S.push_back((T[k])[i]); }
                        U[k] = 1;
                        e = true;
                        break;
                        }
                }
            }
        MTOOLS_INSURE(e == true);
        }
    MTOOLS_INSURE(S.back() == S.front());
    MTOOLS_INSURE(S.back().pos() == iVec2(0, 0));
    return S;
    }


/** end of file */

