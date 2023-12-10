#pragma once


#include "mtools/mtools.hpp" 
using namespace mtools;

#include "Arm.h"
#include "SantaImage.h"
#include "LKHtour.h"
#include "Solution.h"
#include "PotSon.h"
#include "TreeSearch.h"






/*
inline double realScore(std::vector<iVec2>& tour)
    {
    std::vector<double> ord; 

    for (int i = 1; i < tour.size(); i++)
        {
        double di = penaltyL1(tour[i - 1], tour[i]);
        if (di == mtools::INF)
            {
            MTOOLS_ERROR("Tour contains a forbidden moves.");
            }
        ord.push_back(di);
        }
    ord.push_back(distcol(tour[i - 1].pos(), tour[i].pos()));
    
    std::sort(ord.begin(), ord.end());
    double tot = 0; 
    for (auto d : ord)
        {
        tot += d; 
        }
    return tot; 
    }


inline void printRealScore(const char * filename)
    {
    auto V = loadLKHTour(filename); 
    cout << "REAL TOUR SOCRE : " << doubleToWStringHighPrecision(realScore(V));   
    }


    */
std::vector<Arm> reverseSolution(const std::vector<Arm>& V)
    {
    std::vector<Arm> W; 
    W.reserve(V.size()); 
    for (int i = V.size() - 1; i >= 0; i--)
        {
        W.push_back(V[i]);
        }
    return W;
    }


inline void reverseSolution(const char * filename)
    {
    std::vector V = loadSolution(filename);  
    V = reverseSolution(V);
    saveSolution(V, (std::string(filename) + ".rev").c_str());
    return;
    }




/**
* Program to patch 5 tour together
**/
void programPatch()
    {
    std::string fA = arg("part A");
    auto A = loadSolution(fA.c_str());

    std::string fB = arg("part B");
    auto B = loadSolution(fB.c_str());

    std::string fC = arg("part C");
    auto C = loadSolution(fC.c_str());

    std::string fD = arg("part D");
    auto D = loadSolution(fD.c_str());

    std::string fE = arg("part E");
    auto E = loadSolution(fE.c_str());

    std::string out = arg("output");

    auto S = patch(A, B, C, D, E);

    cout << "Solution score : " << score(S, true) << "\n";

    saveSolution(S, out.c_str());
    reverseSolution(S); 
    saveSolution(S, (out + ".rev").c_str());
    cout.getKey();
    }



/**
* Program to compute the score of a tour
**/
void programScore()
    {
    std::string fA = arg("file to score : ");
    auto S = loadSolution(fA.c_str());
    cout << "Solution score : " << score(S, true) << "\n";
    cout.getKey();
    }


/** end of file */

