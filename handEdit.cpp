#include "handEdit.h"







/**
* Class for an unoriented edge
**/
struct Edge
    {

    /** ctor */
    Edge(iVec2 p, iVec2 q, RGBc c = RGBc::c_Black )
        {
        if (id(p) < id(q))
            {
            P = p; 
            Q = q;
            }
        else
            {
            P = q;
            Q = p;
            }
        col = c;
        }


    /**
    * Compare if two edges are equal (do not consider the color). 
    **/
    bool operator==(const Edge& E) const
        {
        return ((P == E.P) && (Q == E.Q));
        }


    /**
    * Total ordering.
    **/
    bool operator<(const Edge& E) const
        {
        return (id(P) < id(E.P)) || ((id(P) == id(E.P)) && (id(Q) < id(E.Q)));
        }


    /**
    * Check if an endpoint belong to an edge. 
    **/
    bool hasEndpoint(iVec2 A) const
        {
        return ((A == P) || (A == Q));
        }

    /* return the id */
    static int id(iVec2 A)
        {
        return 1 + (A.X() + 128) + (A.Y() + 128) * 257;
        }



    iVec2 P, Q;
    RGBc col;


    };








template<typename CANVAS>
void _redrawHandEdit(CANVAS& L, std::set<Edge>& ES, std::vector<iVec2> & V)
    {
    L.clear();
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
            L(mtools::Figure::ThickLine(fVec2(i + 0.5, j - 0.45), fVec2(i + 0.5, j + 0.45), 0.2 * c, true, mtools::RGBc::jetPalette(c)), 3);
            }
        }
    for (int i = -128; i <= 128; i++)
        {
        for (int j = -128; j < 128; j++)
            { // (i,j) <-> (i, j+1)
            const double c = (distcol({ i, j }, { i, j + 1 }) - minc) / (maxc - minc);
            L(mtools::Figure::ThickLine(fVec2(i - 0.45, j + 0.5), fVec2(i + 0.45, j + 0.5), 0.2 * c, true, mtools::RGBc::jetPalette(c)), 3);
            }
        }

    for (auto E : ES)
        {
        L(mtools::Figure::ThickLine((fVec2)E.P, (fVec2)E.Q, 0.2, true, E.col), 0);
        }

    for (int i = 1; i < V.size(); i++)
        {
        L(Figure::Text(mtools::justify_center(mtools::toString(i), 8), (fVec2)V[i], { 1, -1 }, MTOOLS_TEXT_CENTER), 1);
        }


    }



void handEdit(std::vector<iVec2> & V)
    {

    // compute the cut times
    auto CT = cutTimes(V);
    // add the last reversed one
    auto VV = getReversed(V);
    auto CT2 = cutTimes(VV);
    CT.push_back(CutTime(0, 0, (int)(V.size() - CT2[0].n1), (int)V.size(), 0, CT2[0].good, CT2[0].bad));
    // display tour info
    cout << "- tour size  : " << V.size() << " sites\n";
    cout << "- tour score : " << score(V, false) << "\n";
    cout << "cut pos : " << justify_center(toString(CT[0].n0), 6) << " -> " << justify_center(toString(CT[0].n1), 6) << " (" << justify_center(toString(CT[0].n1 - CT[0].n0), 6) << " steps) good: " << justify_center(toString(CT[0].good), 6) << " bad: " << justify_center(toString(CT[0].bad), 6) << "\n";
    cout << "cut neg : " << justify_center(toString(CT2[0].n0), 6) << " -> " << justify_center(toString(CT2[0].n1), 6) << " (" << justify_center(toString(CT2[0].n1 - CT2[0].n0), 6) << " steps) good: " << justify_center(toString(CT2[0].good), 6) << " bad: " << justify_center(toString(CT2[0].bad), 6) << "\n";
    cout << "\n\n";


    // break tour in edges. 
    std::set<Edge> ES;
    for (int i = 1; i < CT[0].n1; i++)
        {
        ES.insert(Edge(V[i - 1], V[i], mtools::RGBc::c_Red));
        }
    for (int i = 1; i <= CT2[0].n1; i++)
        {
        ES.insert(Edge(VV[i - 1], VV[i], mtools::RGBc::c_Blue));
        }
    for (int i = CT[0].n1; i < V.size() - CT2[0].n1; i++)
        {
        RGBc c = Palette::hard_12.get((int64)i, (int64)(CT[0].n1), (int64)(V.size() - CT2[0].n1));
        ES.insert(Edge(V[i - 1], V[i], c));
        }


    auto L = makeFigureCanvas(5);
    _redrawHandEdit(L, ES, V);

    mtools::Plotter2D plotter;
    plotter.setWindowSize(1400, 1200);
    auto PP = mtools::makePlot2DFigure(L);
    plotter[PP];
    plotter.range().setRange({ -130, 130, -130, 130 });
    plotter.gridObject(true);
    plotter.gridObject()->setUnitCells();
    plotter.startPlot();
    
    double dc = 0; 

    while (1)
        {
        double cc = 0;
        for (auto E : ES) { cc += distim(E.P, E.Q); }
        cout << "current cost = " << cc << "\n";
        cout << "current delta = " << dc << "\n";
        cout << "[r] remove edge.\n";
        cout << "[a] add edge.\n";
        cout << "[s] stop and recreate tour.\n";
        int c = cout.getKey();
        if ((c == 'r' || c == 'R'))
            {
            int i1, i2; 
            cout << "\n";
            cout << "id of first point ? "; cout >> i1; cout << i1 << "\n";
            cout << "id of second point ? "; cout >> i2; cout << i2 << "\n";
            cout << "\n";
            iVec2 P = V[i1 % V.size()];
            iVec2 Q = V[i2 % V.size()];
            auto it = ES.find(Edge(P, Q));
            if (it == ES.end())
                {
                cout << "ERROR : EDGE " << P << " <-> " << Q << " DOES NOT EXIST !!!!\n\n"; 
                }
            else
                {
                double w = distim(P, Q);
                dc -= w;
                cout << "weight = " << w << "\n\n";
                ES.erase(it); 
                }
            PP.enable(false);
            _redrawHandEdit(L, ES, V);
            PP.enable(true);
            plotter.redraw(); 
            }
        if ((c == 'a' || c == 'A'))
            {
            int i1, i2;
            cout << "\n";
            cout << "id of first point ? "; cout >> i1; cout << i1 << "\n";
            cout << "id of second point ? "; cout >> i2; cout << i2 << "\n";
            cout << "\n";
            iVec2 P = V[i1 % V.size()];
            iVec2 Q = V[i2 % V.size()];
            auto it = ES.find(Edge(P, Q));
            if (it != ES.end())
                {
                cout << "ERROR : EDGE " << P << " <-> " << Q << " ALREADY EXIST !!!!\n\n";
                } 
            else
                {
                double w = distim(P, Q);
                dc += w;
                cout << "weight = " << w << "\n\n";
                ES.insert(Edge(P, Q, RGBc::c_Green));
                }
            PP.enable(false);
            _redrawHandEdit(L, ES, V);
            PP.enable(true);
            plotter.redraw();
            }

        }


   




    /**
     
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


     
     
    **/


    }

