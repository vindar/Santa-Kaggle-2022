#pragma once

#pragma once

#include <mtools/mtools.hpp>
using namespace mtools;






/**
* Structure containing the informations about a cut time
**/
struct CutTime    
    {
    CutTime(int hp0, int hp1, int N0, int N1, int s, int g, int b) :
        HP0(hp0), HP1(hp1), n0(N0), n1(N1), sign(s), good(g), bad(b)
        {
        }

    CutTime(const CutTime&) = default; 
    CutTime & operator=(const CutTime&) = default;

    std::string toString() const
        {
        mtools::ostringstream oss; 
        oss << "CutTime: " << HP0 << " -> " << HP1 << " [" << ((sign > 0) ? "+1" : "-1") << "]\n";
        oss << " - start time: " << n0 << "\n"; 
        oss << " - end time  : " << n1 << "\n";
        oss << " - good move : " << good << "\n";
        oss << " - bad move  : " << bad << "\n";
        oss << " - tot moves : " << (n1 - n0) << "\n";
        return oss.str(); 
        }

    int HP0;    // half plane at n0,  
    int HP1;    // half plane at n1
    int n0;     // start time
    int n1;     // end point (cut time)
    int sign;   // direction angle (+1 opr -1)
    int good;   // number of angle steps in the "good" direction sign
    int bad;    // number of angle steps in the "bad" direction -sign

    };



/**
*          2
*        3 + 1
*          0
* time to enter a half plane from the oppposte one. 
*/
inline CutTime timeEnter(int startHP, const std::vector<iVec2>& tour, int n0)
    {
    int n = n0;
    int km = 0; 
    int kp = 0;
    switch (startHP)
        {
        case 0: 
            {
            iVec2 P = tour[n++];
            while (n < tour.size()) 
                {
                const iVec2 Q = tour[n]; 
                const int e = (int)(Q - P).X();
                if (e > 0)  kp++; else if (e < 0) km++; 
                if (Q.Y() > 0)
                    {
                    if (Q.X() > 0) return CutTime(0, 1, n0, n, 1, kp, km); else return CutTime(0, 3, n0, n, -1, km, kp);
                    }
                P = Q; 
                n++;
                }
            return CutTime(-1, -1, -1, -1, 0, 0, 0);
            }
        case 1: 
            {
            iVec2 P = tour[n++];
            while (n < tour.size()) 
                {
                const iVec2 Q = tour[n]; 
                const int e = (int)(Q - P).Y();
                if (e > 0)  kp++; else if (e < 0) km++; 
                if (Q.X() < 0)
                    {
                    if (Q.Y() > 0) return CutTime(1, 2, n0, n, 1, kp, km); else return CutTime(1, 0, n0, n, -1, km, kp);
                    }
                P = Q;
                n++;
                }
            return CutTime(-1, -1, -1, -1, 0, 0, 0);
            }
        case 2: 
            {
            iVec2 P = tour[n++];
            while (n < tour.size()) 
                {
                const iVec2 Q = tour[n]; 
                const int e = (int)(Q - P).X();
                if (e > 0)  kp++; else if (e < 0) km++; 
                if (Q.Y() < 0)
                    {
                    if (Q.X() > 0) return CutTime(2, 1, n0, n, -1, kp, km); else return CutTime(2, 3, n0, n, 1, km, kp);
                    }
                P = Q;
                n++;
                }
            return CutTime(-1, -1, -1, -1, 0, 0, 0);
            }
        case 3:
            {
            iVec2 P = tour[n++];
            while (n < tour.size()) 
                {
                const iVec2 Q = tour[n]; 
                const int e = (int)(Q - P).Y();
                if (e > 0)  kp++; else if (e < 0) km++; 
                if (Q.X() > 0)
                    {
                    if (Q.Y() > 0) return CutTime(3, 2, n0, n, -1, kp, km); else return CutTime(3, 0, n0, n, 1, km, kp);
                    }
                P = Q; 
                n++;
                }
            return CutTime(-1, -1, -1, -1, 0, 0, 0);
            }
        }
    MTOOLS_INSURE("IMPOSSIBLE"); 
    return CutTime(-1, -1, -1, -1, 0, 0, 0);
    }





/**
* Construct the vector of primary cut times from a tour. 
**/
inline std::vector<CutTime > cutTimes(const std::vector<iVec2> & tour)
    {
    std::vector<CutTime> CT; 
    int n = 0; 
    int HP = 1; 
    while (1)
        {
        auto C = timeEnter(HP, tour, n);
        if (C.HP0 < 0) return CT; 
        CT.push_back(C); 
        n = C.n1;
        HP = C.HP1;
        }
    }


/** end of file */

