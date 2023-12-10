#pragma once

#include <mtools/mtools.hpp>
using namespace mtools;

#include "Arm.h"



/**
* Compute the score of a solution
**/
double score(const std::vector<Arm>& arm_tour, bool strict);


/**
* Load a solution in kaggle format
**/
std::vector<Arm> loadSolution(const char* filename);


/**
* Save a solution in kaggle format
*/
void saveSolution(const std::vector<Arm>& arm_tour, const char* filename);


/**
* Patch 5 part together to make a tour.
* the order does not matter.
**/
std::vector<Arm> patch(const std::vector<Arm>& A, const  std::vector<Arm>& B, const  std::vector<Arm>& C, const  std::vector<Arm>& D, const  std::vector<Arm>& E);





/** end of file */

