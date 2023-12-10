#pragma once

#include <mtools/mtools.hpp>
using namespace mtools; 

#include "Arm.h"



/**
* Draw an Arm on a canvas. 
**/
template<class CANVAS>
void canvasArm(Arm a, CANVAS & Canvas, int layer, RGBc color_bk = RGBc::c_Blue.getMultOpacity(0.1f), RGBc color_border = RGBc::c_Black)
    {
    mtools::Figure::Group G;
    for (int i = 7; i >= 0; i--)
        {
        iVec2 P = a.centerBox(i);
        cout << " i = " << i << "  | " << P << "\n";
        int l = (i == 0) ? 0 : (1 << (i - 1));
        fBox2 B(P.X() - l - 0.5, P.X() + l + 0.5, P.Y() - l - 0.5, P.Y() + l + 0.5);
        G(Figure::Rectangle(B, color_border, color_bk));
        }
    Canvas(G, layer);
    }


/**
* Draw a circle to indicate a position on a canvas. 
**/
template<class CANVAS>
void canvasPos(iVec2 P, CANVAS& Canvas, int layer, RGBc fill_color = RGBc::c_Red, RGBc border_color = RGBc::c_Black)
    {
    Canvas(Figure::Circle((fVec2)P, 0.4f, border_color, fill_color), layer);
    }




/**
* Display a tour.
*
* Allow 'manual' swaps and return the resulting tour. 
* 
**/
std::vector<iVec2> drawTour(std::vector<iVec2> V);






/**
* Observe the evolution of the solution on a separate thread. 
* V and sol must not have theire buffer changed !
**/
void observe(const std::vector<iVec2>& V, const std::vector<Arm>& sol);



inline void observe(const std::vector<Arm>& sol)
    {
    std::vector<iVec2> V;
    V.reserve(sol.size());
    for (auto a : sol)
        {
        V.push_back(a.pos());
        }
    observe(V, sol);
    }













/** end of file */





