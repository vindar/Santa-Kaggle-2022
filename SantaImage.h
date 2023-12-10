#pragma once

#include <mtools/mtools.hpp>
using namespace mtools;

#define SANTA_IMAGE_LX 257
#define SANTA_IMAGE_LY 257
#define SANTA_IMAGE_CX 128
#define SANTA_IMAGE_CY 128

extern double imR[SANTA_IMAGE_LX * SANTA_IMAGE_LY];
extern double imG[SANTA_IMAGE_LX * SANTA_IMAGE_LY];
extern double imB[SANTA_IMAGE_LX * SANTA_IMAGE_LY];

/*
inline double colR(int x, int y) { return imR[(x + SANTA_IMAGE_CX) + SANTA_IMAGE_LX * (y + SANTA_IMAGE_CY)]; }
inline double colG(int x, int y) { return imG[(x + SANTA_IMAGE_CX) + SANTA_IMAGE_LX * (y + SANTA_IMAGE_CY)]; }
inline double colB(int x, int y) { return imB[(x + SANTA_IMAGE_CX) + SANTA_IMAGE_LX * (y + SANTA_IMAGE_CY)]; }
*/


/**
* Return the "color" distance between two pixels.
**/
inline double distcol(iVec2 P, iVec2 Q)
    {
    auto a1 = (P.X() + SANTA_IMAGE_CX) + SANTA_IMAGE_LX * (P.Y() + SANTA_IMAGE_CY);
    auto a2 = (Q.X() + SANTA_IMAGE_CX) + SANTA_IMAGE_LX * (Q.Y() + SANTA_IMAGE_CY);
    return 3 * (fabs(imR[a1] - imR[a2]) + fabs(imG[a1] - imG[a2]) + fabs(imB[a1] - imB[a2]));
    }


/**
* Return the image distance i.e. sqrt(L1 norm) + colorDist.
**/
inline double distim(iVec2 P, iVec2 Q)
    {
    auto u = abs(P.X() - Q.X()) + abs(P.Y() - Q.Y());
    if (u > 8) return mtools::INF;
    double dl = sqrt((double)u);
    double dc = distcol(P,Q);
    return dl + dc;
    }



/**
* Convert from LKH tour index to image coords
**/
inline mtools::iVec2 id2coord(int id)
    {
    id--;
    const int64_t y = (id / SANTA_IMAGE_LX);
    const int64_t x = id - (y * SANTA_IMAGE_LX);
    return { x - SANTA_IMAGE_CX, y - SANTA_IMAGE_CY };
    }


/**
* Convert from image coord to LKH tour index.
**/
inline int coord2id(mtools::iVec2 P)
    {
    return (int)((P.X() + SANTA_IMAGE_CX) + SANTA_IMAGE_LX * (P.Y() + SANTA_IMAGE_CY));
    }
    
    
/** end of file */

