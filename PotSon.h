#pragma once


#include "mtools/mtools.hpp" 
using namespace mtools;
#include "Arm.h"
#include "distanceArm.h"


/**
* Manage the potential son of an arm. 
**/
class PotSon
    {


    public:

        /**
        * Ctor, set the RNG. 
        **/
        PotSon(MT2004_64  & gen) : _gen(gen)
            {
            _createNeighbour();
            clear();
            }


        /**
        * Remove all selected neighbours.
        **/
        void clear()
            {
            _sel.clear();
            }


        /**
        * Add all the neighbours of arm that move to pixel target 
        * while making a detour of a given length
        * detour: must be between 0,1,2,3,4
        * 
        * Return the number of neighbour really added.
        **/
        int add(Arm arm, iVec2 target, int detour)
            {
            int c = 0; 
            const iVec2 P = arm.pos() - target;
            _moveL1 = (int)(abs(P.X()) + abs(P.Y()));
            const int nb = _moveL1 + 2*detour;
            if (nb > 8) return 0; 
            for (auto& m : _neig[nb])
                {
                const Arm s = arm + m;
                if (s.pos() == target)
                    {
                    _sel.push_back(s);
                    c++;
                    }
                }
            return c; 
            }


        /**
        *  Return the penalty occured by a move with detour index k. 
        **/
        double penaltyL1(int detour)
            {
            if (detour + 2 * detour > 8) return mtools::INF;
            return (sqrt((double)(_moveL1 + 2 * detour)) - sqrt((double)_moveL1));
            }

        /**
        * Return the L1 norm of this jump. 
        **/
        int jumpNormL1()
            {
            return _moveL1; 
            }


        /**
        * Number of selected neighbour.
        **/
        int size()
            {
            return (int)_sel.size();
            }


        /**
        * Return a neighbour according to the probability distribution set with 'setWEights()'
        */
        Arm choice(const double * cdf)
            {
            MTOOLS_INSURE(_sel.size() > 0);
            return _sel[sampleDiscreteRVfromCDF(cdf, _sel.size() - 1, _gen)];
            }


        /**
        * Return a neighbour chosen uniformly
        **/
        Arm unif()
            {
            MTOOLS_INSURE(_sel.size() > 0);
            return _sel[Unif_int(0, _sel.size() - 1, _gen)];
            }


        /**
        * Return a given neighbour, no range check !
        */
        Arm operator[](int i)
            {
            return _sel[i];
            }


        /**
        * Return a reference to the RNG. 
        **/
        MT2004_64& rng()
            {
            return _gen;
            }


    private:

        MT2004_64& _gen; // RNG to use
     
        std::vector<Arm> _sel;          // selected neighour
        
        int _moveL1; 

        // number of neighours depending on the number of non-zero angles. 
        //  1  2   3    4    5    6    7   8
        // 16 112 448 1120 1792 1792 1024 256
        void _createNeighbour()
            {
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
            }

        std::vector<Arm> _neig[9];
    };




    /** end of file */
