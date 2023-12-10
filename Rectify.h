#pragma once


#include "mtools/mtools.hpp" 
using namespace mtools;

#include "Arm.h"


    /**
    * Try to rectify a path to get closer to P at the end. 
    * 
    * Return the number of rectification action performed. 
    **/
    int rectify(std::vector<Arm>& vec, iVec2 P);



/** end of file */
