#pragma once



#include "mtools/mtools.hpp" 
using namespace mtools;

#include "LKHtour.h"



inline std::vector<iVec2> swapTourSub(const std::vector<iVec2> & tour, int i1, int i2, int j1, int j2)
    {

    auto tour2 = getReversed(tour);
    std::map<iVec2, std::vector<iVec2> > ME;
    for (int i = 0; i < tour.size() - 1; i++)
        {
        ME[tour[i]].push_back(tour[i + 1]);
        ME[tour2[i]].push_back(tour2[i + 1]);
        }

    for (int i = 0; i < tour.size(); i++)
        {
        MTOOLS_INSURE(ME[tour[i]].size() == 2);
        }

    iVec2 P1 = tour[i1];
    iVec2 P2 = tour[i2];
    iVec2 Q1 = tour[j1];
    iVec2 Q2 = tour[j2];

    if (ME[P1][0] == P2)
        {
        ME[P1][0] = Q2;
        }
    if (ME[P2][0] == P1)
        {
        ME[P2][0] = Q1;
        }
    if (ME[Q1][0] == Q2)
        {
        ME[Q1][0] = P2;
        }
    if (ME[Q2][0] == Q1)
        {
        ME[Q2][0] = P1;
        }
    if (ME[P1][1] == P2)
        {
        ME[P1][1] = Q2;
        }
    if (ME[P2][1] == P1)
        {
        ME[P2][1] = Q1;
        }
    if (ME[Q1][1] == Q2)
        {
        ME[Q1][1] = P2;
        }
    if (ME[Q2][1] == Q1)
        {
        ME[Q2][1] = P1;
        }

    std::vector<iVec2> V;
    V.reserve(tour.size());

    iVec2 P = tour[0];
    iVec2 Q = tour[1];

    V.push_back(P);
    V.push_back(Q);
    while (Q != tour[0])
        {
        iVec2 R;
        if (ME[Q][0] == P) R = ME[Q][1]; else R = ME[Q][0];
        P = Q;
        Q = R;
        V.push_back(Q);
        }
    return V;
    }



inline std::vector<iVec2> swapTour(const std::vector<iVec2>& tour, int i1, int i2, int j1, int j2, double * cost = nullptr)
    {
    double costI = distim(tour[i1], tour[i2]) + distim(tour[j1], tour[j2]);

    auto V1 = swapTourSub(tour, i1, i2, j1, j2);
    if (V1.size() == tour.size())
        {
        double costA = distim(tour[i1], tour[j2]) + distim(tour[i2], tour[j1]);
        if (cost) { *cost = costA - costI; }
        return V1;
        }
    auto V2 = swapTourSub(tour, i1, i2, j2, j1);
    MTOOLS_INSURE(V2.size() == tour.size());
    double costB = distim(tour[i1], tour[j1]) + distim(tour[i2], tour[j2]);
    if (cost) { *cost = costB - costI; }
    return V2;
    }




/** end of file */