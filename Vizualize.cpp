


#include "mtools/mtools.hpp" 
using namespace mtools;

#include "Vizualize.h"
#include "Arm.h"
#include "LKHTour.h"
#include "cutTime.h"
#include "SwapTour.h"
#include "SantaImage.h"





template<typename CANVAS>
void drawSolCanvas(const std::vector<Arm>& best, const std::vector<iVec2>& tour, CANVAS& L, int val)
    {
    L.clear();
    for (size_t i = 1; i < best.size(); i++)
        {
        L(mtools::Figure::ThickLine((fVec2)best[i - 1].pos(), (fVec2)best[i].pos(), 0.15, true, mtools::Palette::Red_to_Blue((int64)i, (int64)1, (int64)best.size())), 4);
        }

    for (size_t i = 1; i < tour.size(); i++)
        {
        L(mtools::Figure::ThickLine((fVec2)tour[i - 1], (fVec2)tour[i], 0.05, true, mtools::RGBc::c_Gray), 3);
        }

    for (size_t i = 1; i < tour.size(); i++)
        {
        L(Figure::Text(mtools::justify_center(mtools::toString(i), 8), (fVec2)tour[i], { 1, -1 }, MTOOLS_TEXT_CENTER), 1);
        }


    double minc = mtools::INF;
    double maxc = -mtools::INF;
    for (int j = -128; j <= 128; j++)
        {
        for (int i = -128; i < 128; i++)
            { // (i,j) <-> (i+1, j)
            const double c = distcol({ i,j }, { i + 1,j });
            if (minc < c) minc = c;
            if (maxc > c) maxc = c;
            }
        }
    for (int i = -128; i <= 128; i++)
        {
        for (int j = -128; j < 128; j++)
            { // (i,j) <-> (i, j+1)
            const double c = distcol({ i,j }, { i,j +1});
            if (minc < c) minc = c;
            if (maxc > c) maxc = c;
            }
        }
    for (int j = -128; j <= 128; j++)
        {
        for (int i = -128; i < 128; i++)
            { // (i,j) <-> (i+1, j)
            const double c = distcol({ i,j }, { i + 1,j });
            L(mtools::Figure::ThickLine(fVec2(i+0.5, j-0.5), fVec2(i+0.5, j+0.5), 0.1, true, mtools::RGBc::jetPalette(c,minc, maxc)), 3);
            }
        }
    for (int i = -128; i <= 128; i++)
        {
        for (int j = -128; j < 128; j++)
            { // (i,j) <-> (i, j+1)
            const double c = distcol({ i,j }, { i,j + 1 });            
            L(mtools::Figure::ThickLine(fVec2(i - 0.5, j + 0.5), fVec2(i + 0.5, j + 0.5), 0.1, true, mtools::RGBc::jetPalette(c, minc, maxc)), 3);
            }
        }



    if ((val < 0) || (val >= best.size())) val = (int)best.size() - 1; 
    canvasArm(best[val], L, 2);

    mtools::Figure::Group G;
    G(Figure::BoxRegion(fBox2(-140, 140, -140, 140), RGBc::c_Black));
    G(Figure::BoxRegion(fBox2(-128.5, 128.5, -128.5, 128.5), RGBc::c_White));
    L(G, 0);
    }





void _observe(const std::vector<iVec2> * V, const std::vector<Arm> * sol)
    {
    Console cons("Observe Tour console");
    cons.move(50, 800);
    Plotter2D plotter = new Plotter2D;
    //plotter.setWindowSize(1150, 1000);

    auto L = makeFigureCanvas(5);

    drawSolCanvas(*sol, *V, L, -1);

    auto PL = mtools::makePlot2DFigure(L, 1);

    plotter.setDrawingSize(1200, 1000);
    PL.showLayer(1, false);
    plotter[PL];

    plotter.gridObject(true);
    plotter.gridObject()->setUnitCells();
    plotter.range().setRange({ -130, 130, -130, 130 });
    plotter.startPlot();
    while (1)
        {
        std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        cons << "Position to check (-1 for end) ? ";
        std::string s;
        cons >> s;
        cons << s << "\n\n";
        if (s.size() > 0)
            {
            int val;
            fromString(s, val);
            PL.suspend(true);
            drawSolCanvas(*sol, *V, L, val);
            PL.suspend(false);
            }
        plotter.redraw();
        }
    }







void observe(const std::vector<iVec2>& V, const std::vector<Arm>& sol)
    {    
    new std::thread(_observe, &V, &sol);
    std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::this_thread::yield();
    }








void _drawTour(mtools::FigureCanvas<5> & L, const std::vector<iVec2> & V)
    {
    // compute the cut times
    auto CT = cutTimes(V);

    // add the last reversed one
    auto VV = getReversed(V);
    auto CT2 = cutTimes(VV);
    CT.push_back(CutTime(0, 0, (int)(V.size() - CT2[0].n1), (int)V.size(), 0, CT2[0].good, CT2[0].bad));

    // display tour info


    L.clear();
    iVec2 prev = V[0];

    for (int i = 1; i < CT[0].n1; i++)
        {
        L(mtools::Figure::ThickLine((fVec2)V[i - 1], (fVec2)V[i], 0.2, true, mtools::RGBc::c_Red), 0);
        }
    for (int i = 1; i <= CT2[0].n1; i++)
        {
        L(mtools::Figure::ThickLine((fVec2)VV[i - 1], (fVec2)VV[i], 0.2, true, mtools::RGBc::c_Blue), 0);
        }

    for (int i = CT[0].n1; i < V.size() - CT2[0].n1; i++)
        {
        RGBc c = Palette::hard_12.get((int64)i, (int64)(CT[0].n1), (int64)(V.size() - CT2[0].n1));
        L(mtools::Figure::ThickLine((fVec2)V[i - 1], (fVec2)V[i], 0.2, true, c), 0);
        }

    for (int i = 1; i < V.size(); i++)
        {
        L(Figure::Text(mtools::justify_center(mtools::toString(i), 8), (fVec2)V[i], { 1, -1 }, MTOOLS_TEXT_CENTER), 1);
        }



    double minc = mtools::INF;
    double maxc = -mtools::INF;
    for (int j = -128; j <= 128; j++)
        {
        for (int i = -128; i < 128; i++)
            { // (i,j) <-> (i+1, j)
            const double c = distcol({ i,j }, { i + 1,j });
            if (minc > c) minc = c;
            if (maxc < c) maxc = c;
            }
        }
    for (int i = -128; i <= 128; i++)
        {
        for (int j = -128; j < 128; j++)
            { // (i,j) <-> (i, j+1)
            const double c = distcol({ i,j }, { i,j + 1 });
            if (minc > c) minc = c;
            if (maxc < c) maxc = c;
            }
        }
    for (int j = -128; j <= 128; j++)
        {
        for (int i = -128; i < 128; i++)
            { // (i,j) <-> (i+1, j)
            const double c = (distcol({ i,j }, { i + 1,j }) - minc) / (maxc - minc);
            L(mtools::Figure::ThickLine(fVec2(i + 0.5, j - 0.45), fVec2(i + 0.5, j + 0.45), 0.1 * c, true, mtools::RGBc::jetPalette(c)), 3);
            }
        }
    for (int i = -128; i <= 128; i++)
        {
        for (int j = -128; j < 128; j++)
            { // (i,j) <-> (i, j+1)
            const double c = (distcol({ i,j }, { i + 1,j }) - minc) / (maxc - minc);
            L(mtools::Figure::ThickLine(fVec2(i - 0.45, j + 0.5), fVec2(i + 0.45, j + 0.5), 0.1 * c, true, mtools::RGBc::jetPalette(c)), 3);
            }
        }

    }



std::vector<iVec2> drawTour(std::vector<iVec2> V)
    {
    Console cons;
    auto L = mtools::makeFigureCanvas(4);
    mtools::Plotter2D plotter;
    plotter.setWindowSize(1200, 1000);
    auto P = mtools::makePlot2DFigure(L);
    plotter[P];
    plotter.range().setRange({ -130, 130, -130, 130 });
    plotter.gridObject(true);
    plotter.gridObject()->setUnitCells();

    _drawTour(L, V);
        
    plotter.redraw();
    cout << "Displaying a tour\n";
    cout << "-----------------\n\n";
    cout << "- Zoom in/out to examine the tour (with arrow keys and mouse on the plotter windows)\n";
    cout << "- Close the plotter windows (not this one!) to continue...\n";
    cout << "\n\n";
    cout << "-> tour size  : " << V.size() << " sites\n";
    cout << "-> tour score : " << score(V, false) << "\n";
    cout << "\n\n";
    plotter.plot();
    return V; 
    }








/** end of file */