#pragma once


#include <mtools/mtools.hpp>
using namespace mtools;

#include "SantaImage.h"


/**
* Compute the score of a Pixel tour.
*/
double score(const std::vector<iVec2>& tour, bool strict = false);


/**
* Load a tour file output from LKH.
* -> starts and finishes at the origin. 
*/
std::vector<mtools::iVec2> loadLKHTour(const std::string filename);


/**
* Save a tour in LKH file format
**/
void saveLKHTour(std::vector<mtools::iVec2>& tour, const std::string filename);



/**
* Split a tour in five part.
* A = (0,0) -> Coroner 1
* B = Corner 1 -> Corner 2
* C = Corner 2 -> Corner 3
* D = Corner 3 -> Corner 4
* E = (0,0) -> corner 4    [NOTE : this one is reversed !]
**/
void splitTour(const std::vector<mtools::iVec2>& tour,
    std::vector<mtools::iVec2>& A,
    std::vector<mtools::iVec2>& B,
    std::vector<mtools::iVec2>& C,
    std::vector<mtools::iVec2>& D,
    std::vector<mtools::iVec2>& E
    );



/**
* Return a reversed tour.
**/
template<typename T>
std::vector<T> getReversed(const std::vector<T>& tour)
    {
    std::vector<T> res;
    res.reserve(tour.size());
    for (int i = ((int)tour.size()) - 1; i >= 0; i--)
        {
        res.push_back(tour[i]);
        }
    return res;
    }





/**
* Return no zero if we are at a corner.
*/
inline int isCorner(iVec2 P)
    {
    if (P == iVec2(SANTA_IMAGE_CX, SANTA_IMAGE_CY)) return 1;
    if (P == iVec2(-SANTA_IMAGE_CX, SANTA_IMAGE_CY)) return 2;
    if (P == iVec2(SANTA_IMAGE_CX, -SANTA_IMAGE_CY)) return 3;
    if (P == iVec2(-SANTA_IMAGE_CX, -SANTA_IMAGE_CY)) return 4;
    return 0;
    }



/**
* Print the corner positions
**/
void cornerTour(const std::vector<mtools::iVec2>& tour);





/** end of file */


