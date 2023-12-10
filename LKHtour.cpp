
#include "LKHtour.h"
#include "SantaImage.h"





double score(const std::vector<iVec2>& tour, bool strict)
    {
    double score = 0;
    iVec2 P = tour[0];
    for (int i = 1; i < tour.size(); i++)
        {
        iVec2 Q = tour[i];

        double pen = distim(P, Q);
        if (strict)
            {
            if (pen == mtools::INF)
                {
                MTOOLS_ERROR("Tour contains a forbidden move.");
                }
            }
        score += pen;
        P = Q;
        }

    if (strict)
        {
        if (tour[0] != iVec2(0, 0))
            {
            MTOOLS_ERROR("tour does not start at {0,0}");
            }
        if (tour.back() != iVec2(0, 0))
            {
            MTOOLS_ERROR("tour does not end at {0,0}");
            }

        std::set<iVec2> ss;

        for (auto Q : tour)
            {
            ss.insert(Q);
            if ((Q.X() < -128) || (Q.X() > 128))
                {
                MTOOLS_ERROR("X value out of range [-128,128]");
                }
            if ((Q.Y() < -128) || (Q.Y() > 128))
                {
                MTOOLS_ERROR("Y value out of range [-128,128]");
                }
            }
        if (ss.size() != 257 * 257)
            {
            MTOOLS_ERROR("Tour does not visit all points !");
            }
        }
    return score;
    }



void splitTour(const std::vector<mtools::iVec2>& tour,
    std::vector<mtools::iVec2>& A,
    std::vector<mtools::iVec2>& B,
    std::vector<mtools::iVec2>& C,
    std::vector<mtools::iVec2>& D,
    std::vector<mtools::iVec2>& E)
    {
    int i = 0; 

    A.clear();
    A.push_back(tour[i++]);
    while (i < tour.size())
        {
        A.push_back(tour[i]);
        if (isCorner(tour[i]) != 0) break;
        i++;
        }
    if (i == tour.size()) MTOOLS_ERROR("splitTour() MISSING CORNER 1");

    B.clear();
    B.push_back(tour[i++]);
    while (i < tour.size())
        {
        B.push_back(tour[i]);
        if (isCorner(tour[i]) != 0) break;
        i++;
        }
    if (i == tour.size()) MTOOLS_ERROR("splitTour() MISSING CORNER 2");

    C.clear();
    C.push_back(tour[i++]);
    while (i < tour.size())
        {
        C.push_back(tour[i]);
        if (isCorner(tour[i]) != 0) break;
        i++;
        }
    if (i == tour.size()) MTOOLS_ERROR("splitTour() MISSING CORNER 3");

    D.clear();
    D.push_back(tour[i++]);
    while (i < tour.size())
        {
        D.push_back(tour[i]);
        if (isCorner(tour[i]) != 0) break;
        i++;
        }
    if (i == tour.size()) MTOOLS_ERROR("splitTour() MISSING CORNER 4");

    E.clear();
    while (i < tour.size())
        {
        E.push_back(tour[i]);
        i++;
        }
    E = getReversed(E); 
    return; 
    }



std::vector<mtools::iVec2> loadLKHTour(const std::string filename)
    {
    std::vector<mtools::iVec2> vec;
    auto S = mtools::loadStringFromFile(filename);
    if (S.size() == 0)
        {
        MTOOLS_ERROR(std::string("Cannot open :") << filename);
        }
    auto V = mtools::tokenize(S, "", "\n \t");
    size_t i = 0;
    while (i < V.size())
        {
        auto s = V[i];
        if (strcmp(s.c_str(), "TOUR_SECTION") == 0) break;
        i++;
        }
    if (i == V.size())
        {
        MTOOLS_ERROR(std::string("No TOUR_SECTION found.."));
        }
    int ori = -1; 
    for (i; i < V.size(); i++)
        {
        auto s = V[i];
        int v;
        mtools::fromString(s, v);
        if (v >= 1)
            {
            auto C = id2coord(v);
            if (C == iVec2(0, 0))
                {
                if (ori != -1) MTOOLS_ERROR(std::string("Multiple ORIGIN found.."));
                ori = (int)vec.size();
                }
            vec.push_back(C);
            }
        if (v == -1) break;
        }
    if (ori == -1)
        {
        MTOOLS_ERROR(std::string("No ORIGIN found.."));
        }
    std::vector<mtools::iVec2> vec2;
    vec2.reserve(vec.size()+ 1);
    const int l = (int)vec.size();
    for (int i = 0; i < l + 1; i++)
        {
        vec2.push_back(vec[(i + ori) % l]);
        }
    return vec2;
    }


void saveLKHTour(std::vector<mtools::iVec2> & tour, const std::string filename)
    {
    LogFile f(filename, false, false, false);
    f << "TOUR_SECTION\n";
    for (auto P : tour)
        {
        f << coord2id(P) << "\n";
        }
    f << "-1\n";
    f << "EOF";
    }



void cornerTour(const std::vector<mtools::iVec2>& tour)
    {
    for (size_t i = 1; i < tour.size(); i++)
        {
        auto P = tour[i];
        if ((P == iVec2(128, 128)) || (P == iVec2(-128, 128)) || (P == iVec2(128, -128)) || (P == iVec2(-128, -128)))
            {
            cout << "Corner (" << P.X() << "," << P.Y() << ") at index " << i << "\n";            
            }
        }

    }






/** end of file */

