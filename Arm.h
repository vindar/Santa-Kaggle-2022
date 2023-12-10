#pragma once


#include "mtools/mtools.hpp" 
using namespace mtools;

#include "SantaImage.h"



/** defined in arm2reachArray.cpp */
std::pair<int, int> p2reach(int arm_index, int64 pos);



/**
 * 
 * An arm configuration. Same size as uint64_t (8 bytes). 
 * 
 **/
class Arm
    {


    friend Arm operator+(const Arm& arm1, const Arm& arm2);
    friend Arm operator-(const Arm& arm1, const Arm& arm2);

    /**
    * Return the cost of moving from a to b (assuming they are admissible)
    **/
    friend double penaltyL1(Arm a, Arm b);


    /**
    * Return the loss of moving from a to b compared to the sqrt(L1) norm on the pixel. 
    **/
    friend double lossL1(Arm a, Arm b);


public:
    
    static const uint64_t START_ARM_POS = 11533731900201661375; 


    /**
    * ctor. Set the position to the challenge start position.
    */
    Arm() : _val(START_ARM_POS)
        {
        _free = 0;
        }


    /**
    * ctor. P must be either the origin or a corner. 
    */
    Arm(iVec2 P)
        {
        if (P == iVec2(0, 0))
            {
            reset();
            return;
            }
        int a = 0, b = 0; 
        if (P == iVec2(-128, -128))
            {
            a = -1; b = -1; 
            }
        if (P == iVec2(128, -128))
            {
            a = 1; b = -1;
            }
        if (P == iVec2(-128, 128))
            {
            a = -1; b = 1;
            }
        if (P == iVec2(128, 128))
            {
            a = 1; b = 1;
            }

        if ((a == 0) || (b == 0))
            {
            MTOOLS_ERROR("Arm::Arm(iVec2 P) WRONG POSITION !");
            }
        setPos(0, { a*1, b * 1 });
        setPos(1, { a*1, b * 1 });
        setPos(2, { a * 2, b * 2 });
        setPos(3, { a * 4, b * 4 });
        setPos(4, { a * 8, b * 8 });
        setPos(5, { a * 16, b * 16 });
        setPos(6, { a * 32, b * 32 });
        setPos(7, { a * 64, b * 64 });
        _free = 0;
        MTOOLS_INSURE(pos() == P);
        }


    /**
    * Set the angle of each arm. (0 = smallest, 7 = largest). 
    **/
    Arm(int v0, int v1, int v2, int v3, int v4, int v5, int v6, int v7)
        {
        _A0 = v0;
        _A1 = v1;
        _A2 = v2;
        _A3 = v3;
        _A4 = v4;
        _A5 = v5;
        _A6 = v6;
        _A7 = v7;
        _free = 0; 
        }


    /**
    * Constructor from a string in the submission format e.g. "64 13;29 -32;5 16;0 -8;-4 0;-1 -2;-1 0;1 -1"
    **/
    Arm(const std::string& str)
        {
        parse(str);
        }


    /**
    * Default copy ctor
    **/
    Arm(const Arm&) = default; 


    /**
    * Default assignement operator
    **/
    Arm& operator=(const Arm&) = default; 


    /**
    * Comparison (equal if all arm are the same). 
    **/
    bool operator==(const Arm& arm) const
        {
        return ((_A0 == arm._A0) && (_A1 == arm._A1) && (_A2 == arm._A2) && (_A3 == arm._A3) && (_A4 == arm._A4) && (_A5 == arm._A5) && (_A6 == arm._A6) && (_A7 == arm._A7));
        }

    /**
    * Comparison (equal if all arm are the same).
    **/
    bool operator!=(const Arm& arm) const
        {
        return !(operator==(arm));
        }



    /**
    * Arm addition.
    **/
    void operator+=(const Arm& arm)
        {
        _A0 = _A0 + arm._A0;
        _A1 = _A1 + arm._A1;
        _A2 = _A2 + arm._A2;
        _A3 = _A3 + arm._A3;
        _A4 = _A4 + arm._A4;
        _A5 = _A5 + arm._A5;
        _A6 = _A6 + arm._A6;
        _A7 = _A7 + arm._A7;
        }

    /**
    * Arm substraction
    **/
    void operator-=(const Arm& arm)
        {
        _A0 = _A0 - arm._A0;
        _A1 = _A1 - arm._A1;
        _A2 = _A2 - arm._A2;
        _A3 = _A3 - arm._A3;
        _A4 = _A4 - arm._A4;
        _A5 = _A5 - arm._A5;
        _A6 = _A6 - arm._A6;
        _A7 = _A7 - arm._A7;
        }

    /**
    * Set all arm values to 0. 
    **/
    void setZero()
        {
        _val = 0; 
        _free = 0;
        }



    /**
    * Set the arm to the challenge starting position
    **/
    void reset()
        {
        _val = START_ARM_POS;
        _free = 0;
        }


    /**
    * Get the uint64_t representation of the object
    **/
    uint64_t val() const
        {
        return _val;
        }


    /**
    * Set the uint64_t representation of the object
    **/
    void setVal(uint64_t val)
        {
        _val = val;
        _free = 0; 
        }


    /**
     * Final position pointed by the arms in [-128,128]x[-128,28]
     */
    iVec2 pos() const
        {
        return _getArmPos(1, _A0) + _getArmPos(1, _A1) + _getArmPos(2, _A2) + _getArmPos(4, _A3) + _getArmPos(8, _A4) + _getArmPos(16, _A5) + _getArmPos(32, _A6) + _getArmPos(64, _A7);
        }


    /**
     * Position of the tip of a given arm (centered at 0). 
     */
    iVec2 pos(int arm_index) const
        {
        switch (arm_index)
            {
            case 0: return _getArmPos(1, _A0);
            case 1: return _getArmPos(1, _A1);
            case 2: return _getArmPos(2, _A2);
            case 3: return _getArmPos(4, _A3);
            case 4: return _getArmPos(8, _A4);
            case 5: return _getArmPos(16, _A5);
            case 6: return _getArmPos(32, _A6);
            case 7: return _getArmPos(64, _A7);
            }        
        MTOOLS_ERROR("Arm::pos(). Invalid arm number: " << arm_index);
        return { 0,0 };
        }


    /**
    * Position of the center of the ith rectangle 
    * i.e.  sum of the positon of the arm in [arm_index,7]. 
    */
    iVec2 centerBox(int arm_index)
        {
        iVec2 P(0, 0);
        int i = 7; 
        while (i >= arm_index)
            {
            P += pos(i);
            i--; 
            }
        return P; 
        }



    /**
     * Set the position of the tip of a given arm (centered at 0).
     */
    void setPos(int arm_index, iVec2 P) 
        {        
        const int l = lenArm(arm_index);
        const int x = (int)P.X();
        const int y = (int)P.Y();
        MTOOLS_ASSERT((x >= -l) && (x <= l));
        MTOOLS_ASSERT((y >= -l) && (y <= l));
        if (x == -l) setAngle(arm_index, 6*l + l-y);
        else if (x == l) setAngle(arm_index, 2*l + y+l);
        else if (y == -l) setAngle(arm_index, x+l);
        else if (y == l) setAngle(arm_index, 4*l + l-x);
        else MTOOLS_ERROR("Arm::setPos(). Invalid pos");
        }




    /**
    * Angle of the tip of a given arm
    * 
    *    6l---------4l
    *    |          |
    *    |          |
    *    |          |
    *    0----------2l
    */
    int angle(int arm_index) const 
        {
        switch (arm_index)
            {
            case 0: return _A0; 
            case 1: return _A1;
            case 2: return _A2;
            case 3: return _A3;
            case 4: return _A4;
            case 5: return _A5;
            case 6: return _A6;
            case 7: return _A7;
            }
        MTOOLS_ERROR("Arm::angle(). Invalid arm number: " << arm_index);
        return 0; 
        }


    /**
    * Set the angle of the tip of a given arm
    *
    *    6l---------4l
    *    |          |
    *    |          |
    *    |          |
    *    0----------2l
    */
    void setAngle(int arm_index, int val)
        {
        switch (arm_index)
            {
            case 0: _A0 = val; return;
            case 1: _A1 = val; return;
            case 2: _A2 = val; return;
            case 3: _A3 = val; return;
            case 4: _A4 = val; return;
            case 5: _A5 = val; return;
            case 6: _A6 = val; return;
            case 7: _A7 = val; return;
            }
        MTOOLS_ERROR("Arm::setAngle(). Invalid param arm: " << arm_index << "  val: " << val);
        }


    /**
    * Move a the angle of a given arm 
    **/
    void addAngle(int arm_index, int val)
        {
        switch (arm_index)
            {
            case 0: _A0 = (_A0 + val); return;
            case 1: _A1 = (_A1 + val); return;
            case 2: _A2 = (_A2 + val); return;
            case 3: _A3 = (_A3 + val); return;
            case 4: _A4 = (_A4 + val); return;
            case 5: _A5 = (_A5 + val); return;
            case 6: _A6 = (_A6 + val); return;
            case 7: _A7 = (_A7 + val); return;
            }
        MTOOLS_ERROR("Arm::addAngle(). Invalid param arm: " << arm_index << "  val: " << val);
        }




    /**
    * Return the length of a given arm: 1 1 2 4 8 16 32 64
    */
    int lenArm(int arm_index) const
        {
        return (arm_index == 0) ? 1 : (1 << (arm_index - 1));
        }


    /**
    * Print info about the object into a string.
    */
    std::string toString() const
        {
        mtools::ostringstream os; 
        auto P = pos();
        os << "Arm (" << P.X() << " , " << P.Y() << ")   -> val =" << _val << "\n";
        for (int i = 0; i < 8; i++) 
            {
            const int l = lenArm(i);
            const int x = (int)pos(i).X();
            const int y = (int)pos(i).Y();
            if (x == -l) os << "-"; else if (x == +l) os << "+"; else os << ".";
            if (y == -l) os << "-"; else if (y == +l) os << "+"; else os << ".";
            os << " \t";
            os << "[" << lenArm(i) << "]\t angle=" << angle(i) << "\t pos = (" << x << ", " << y << ")\n";
            }
        return os.str();
        }



    iVec2 lock(int arm_index)
        {
        const int l = lenArm(arm_index);
        const int x = (int)pos(arm_index).X();
        const int y = (int)pos(arm_index).Y();
        iVec2 P; 
        P.X() = (x == -l) ? -1 : ((x == l) ? 1 : 0);
        P.Y() = (y == -l) ? -1 : ((y == l) ? 1 : 0);
        return P; 
        }




    /**
    * Print the arm configuration in the submission format
    */
    std::string str() const
        {
        mtools::ostringstream os;
        for (int i = 7; i >= 0; i--)
            {
            os << pos(i).X() << " " << pos(i).Y() << ((i == 0) ? "\n" : ";");
            }
        return os.str();
        }


    /**
    * Parse from a string in the submission format e.g. "64 13;29 -32;5 16;0 -8;-4 0;-1 -2;-1 0;1 -1"
    **/
    bool parse(const std::string & str)
        {
        auto R = tokenize(str, "", "; \n");
        if (R.size() != 16)
            {
            reset(); 
            return false;
            }
        for (int i = 0; i < 8; i++)
            {
            int x, y;
            fromString(R[2 * i], x);
            fromString(R[2 * i + 1], y);
            setPos(7 - i, iVec2(x, y));
            }
        return true;
        }


    /**
    * The center of the box of arm k = pos(k) + pos(k+1) + ... + pos(7)
    **/
    iVec2 centerBox(int arm_index) const
        {
        iVec2 C(0, 0);
        for (int k = 7; k >= arm_index; k--) { C += pos(k); }
        return C;
        }


    /**
    * Return the bounding box associated with arm k 
    * (ie for k= 7 it is a quarter image and for k =0 a single pixel). 
    */
    iBox2 boundingBox(int arm_index) const
        {
        iVec2 C = centerBox(arm_index);
        const int64 r = (arm_index == 0) ? 0 : (1 << (arm_index - 1));
        return iBox2(C.X() - r, C.X() + r, C.Y() - r, C.Y() + r);
        }





    /**
    * Compute the two angles that an arm with a given index must rotate in order for 
    * its bounding box to contain point P. 
    * 
    * The value return are in ]-4*la, 4*la] where  la = lenArm(index)
    * 
    * Set error to signify when an error occur (either a special move or a larger arm 
    * does not contain P). 
    **/
    std::pair<int, int> anglesToReach(iVec2 P, int arm_index, bool & error) const
        {
        error = false; 
        if (arm_index == 0)
            {
            iVec2 Q = P - centerBox(1);
            if (std::max(abs(Q.X()), abs(Q.Y())) != 1)
                { // special case 
                error = true; 
                return {0,0};
                }
            const int tab[9] = { 0,1,2,7,-1,3,6,5,4 };
            const int ta = tab[(Q.X() + 1) + 3 * (Q.Y() + 1)];
            const int a = angle(0);
            int r = (ta - a) % 8;
            if (r <= -4 ) r += 8 ;
            if (r > 4) r -= 8;
            return { r, r };
            }
        iVec2 Q = P - centerBox(arm_index + 1);
        const int la = lenArm(arm_index);
        if ((abs(Q.X()), abs(Q.Y())) > 2 * la)
            {
            error = true; 
            return { 0, 0 };
            }
        Q += iVec2(la * 2, la * 2);
        auto mm = p2reach(arm_index, Q.X() + (4 * la + 1) * Q.Y());
        int n, p;
        int a = angle(arm_index);
        if (mm.first <= mm.second)
            {
            if (a < mm.first)
                {
                p = mm.first - a;
                n = (8 * la) - p - (mm.second - mm.first);
                } else if (a > mm.second)
                    {
                    n = a - mm.second;
                    p = (8 * la) - n - (mm.second - mm.first);
                    } else
                    {
                    n = 0;
                    p = 0;
                    }
            } else
            {
            if ((a > mm.second) && (a < mm.first))
                {
                n = a - mm.second;
                p = mm.first - a;
                } else
                {
                n = 0;
                p = 0;
                }
            }

        p = p % (8 * la);
        if (p <= -4 * la) p += 8 * la;
        if (p > 4 * la) p -= 8 * la;

        n = (-n) % (8 * la);
        if (n <= -4 * la) n += 8 * la;
        if (n > 4 * la) n -= 8 * la;

        return { p, n };
        }



    /**
    * Return the 128 extremal path to reach point P from this arm. 
    * All the arm 'a' in the array are located at P. 
    * They may not be all distinct. 
    **/
    std::array<Arm, 128> pathToReach(iVec2 P) const
        {
        std::array<Arm, 128> v;
        if (centerBox(1) == P)
            {
            MTOOLS_ERROR("This case must be treated separately !");
            return v;
            }
        int nb = 0;
        _pathToReach(P, 7, v, nb);
        MTOOLS_INSURE(nb = 128);
        return v;
        }




    /**
    * Check if this arm is a valid step increment (rotation -1, 0 or +1 on each arm) 
    **/
    bool isValidStep() const
        {
        if ((_A0 != 0) && (_A0 != 1) && (_A0 != 7)) return false; 
        if ((_A1 != 0) && (_A1 != 1) && (_A1 != 7)) return false;
        if ((_A2 != 0) && (_A2 != 1) && (_A2 != 15)) return false;
        if ((_A3 != 0) && (_A3 != 1) && (_A3 != 31)) return false;
        if ((_A4 != 0) && (_A4 != 1) && (_A4 != 63)) return false;
        if ((_A5 != 0) && (_A5 != 1) && (_A5 != 127)) return false;
        if ((_A6 != 0) && (_A6 != 1) && (_A6 != 255)) return false;
        if ((_A7 != 0) && (_A7 != 1) && (_A7 != 511)) return false;
        return true;
        }


    /**
    * Return the sign of the angle of an arm (+1, -1 or 0)
    * (sign is positive at exact half by convention)
    **/
    int sign(int arm_index) const
        {
        const int a = angle(arm_index);
        if (a == 0) return 0; 
        const int la = lenArm(arm_index);
        return ((a > 4 * la) ? -1 : 1);
        }



    private:



    union 
        {
        uint64_t _val;

        struct 
            {
            uint64_t
                _A0 : 3,
                _A1 : 3,
                _A2 : 4,
                _A3 : 5,
                _A4 : 6,
                _A5 : 7,
                _A6 : 8,
                _A7 : 9,
                _free  : 19;
            };
        };



    /** helper method to compute the position of an arm from its angle */
    iVec2 _getArmPos(int l, int a) const
        {
        if (a < 2*l) return { -l + a, -l };
        a -= 2*l;
        if (a < 2*l) return { l , -l + a };
        a -= 2*l;
        if (a < 2*l) return { l - a , l };
        a -= 2*l;
        return { -l , l - a };
        }


    /** recursive method called by path2reach */
    /*
    void _pathToReach(iVec2 P, int arm_index, std::array<Arm, 128>& V, int& nb) const
        {
        if (arm_index == 0)
            {
            iVec2 Q = P - centerBox(1);
            MTOOLS_INSURE(std::max(abs(Q.X()), abs(Q.Y())) == 1);
            const int tab[9] = { 0,1,2,7,-1,3,6,5,4 };
            const int ta = tab[(Q.X() + 1) + 3 * (Q.Y() + 1)];
            const int a = angle(0);
            Arm b = (*this);
            b.addAngle(0, (ta - a));
            MTOOLS_INSURE(b.pos() == P);
            V[nb++] = b;
            return;
            }
        iVec2 Q = P - centerBox(arm_index + 1);
        const int la = lenArm(arm_index);
        MTOOLS_INSURE(std::max(abs(Q.X()), abs(Q.Y())) <= 2 * la);
        Q += iVec2(la * 2, la * 2);
        auto mm = p2reach(arm_index, Q.X() + (4 * la + 1) * Q.Y());
        int n, p;
        int a = angle(arm_index);
        if (mm.first <= mm.second)
            {
            if (a < mm.first)
                {
                p = mm.first - a;
                n = (8 * la) - p - (mm.second - mm.first);
                } else if (a > mm.second)
                    {
                    n = a - mm.second;
                    p = (8 * la) - n - (mm.second - mm.first);
                    } else
                    {
                    n = 0;
                    p = 0;
                    }
            } else
            {
            if ((a > mm.second) && (a < mm.first))
                {
                n = a - mm.second;
                p = mm.first - a;
                } else
                {
                n = 0;
                p = 0;
                }
            }

            {
            Arm b = (*this);
            b.addAngle(arm_index, p);
            b._pathToReach(P, arm_index - 1, V, nb);
            }
            {
            Arm b = (*this);
            b.addAngle(arm_index, -n);
            b._pathToReach(P, arm_index - 1, V, nb);
            }
        }
    */





    void _pathToReach(iVec2 P, int arm_index, std::array<Arm, 128>& V, int& nb) const
        {
        bool err = false; 
        auto R = anglesToReach(P, arm_index, err);
        MTOOLS_INSURE(err == false);
        if (arm_index == 0)
            {
            Arm b = (*this);
            b.addAngle(0, R.first);
            MTOOLS_INSURE(b.pos() == P);
            V[nb++] = b;
            return;
            }
        {
        Arm b = (*this);
        b.addAngle(arm_index, R.first);
        b._pathToReach(P, arm_index - 1, V, nb);
        }
        {
        Arm b = (*this);
        b.addAngle(arm_index, R.second);
        b._pathToReach(P, arm_index - 1, V, nb);
        }
        }










 







        


    };





struct compareArm
    {
    bool operator()(Arm a, Arm b) const 
        {
        return (a.val() < b.val());
        }
    };





inline Arm operator+(const Arm& arm1, const Arm& arm2)
    {
    return Arm((int)(arm1._A0 + arm2._A0), 
        (int)(arm1._A1 + arm2._A1),
        (int)(arm1._A2 + arm2._A2),
        (int)(arm1._A3 + arm2._A3),
        (int)(arm1._A4 + arm2._A4),
        (int)(arm1._A5 + arm2._A5),
        (int)(arm1._A6 + arm2._A6),
        (int)(arm1._A7 + arm2._A7));
    }


inline Arm operator-(const Arm& arm1, const Arm& arm2)
    {
    return Arm((int)(arm1._A0 - arm2._A0), 
        (int)(arm1._A1 - arm2._A1),
        (int)(arm1._A2 - arm2._A2),
        (int)(arm1._A3 - arm2._A3),
        (int)(arm1._A4 - arm2._A4),
        (int)(arm1._A5 - arm2._A5),
        (int)(arm1._A6 - arm2._A6),
        (int)(arm1._A7 - arm2._A7));
    }



/** end of file */











    /*
    void _pathToReach(iVec2 P, int arm_index, std::array<Arm, 256> & V, int& nb) const
        {
        if (arm_index < 7)
            {
            MTOOLS_INSURE(boundingBox(arm_index + 1).isInside(P));
            }

            {
            Arm b = (*this);
            int i = 0;
            while (!(b.boundingBox(arm_index).isInside(P)))
                {
                b.addAngle(arm_index, 1);
                i++;
                MTOOLS_INSURE(i <= 512);
                }
            // ok recurse
            if (arm_index == 0)
                {
                MTOOLS_INSURE(b.pos() == P);
                V[nb++] = b;
                }
            else
                {
                b._pathToReach(P, arm_index - 1, V, nb);
                }
            i += 1;
            }


            {
            Arm b = (*this);
            int i = 0;
            while (!(b.boundingBox(arm_index).isInside(P)))
                {
                b.addAngle(arm_index, -1);
                i++;
                MTOOLS_INSURE(i < 512);
                }
            // ok recurse
            if (arm_index == 0)
                {
                MTOOLS_INSURE(b.pos() == P);
                V[nb++] = b;
                }
            else
                {
                b._pathToReach(P, arm_index - 1, V, nb);
                }
            }
        }
*/


/**
* Return all the extremal config at pixel P (seen from a).
*
*
**/

/*
std::array<Arm, 256> pathToReach(iVec2 P) const
    {
    std::array<Arm, 256> v;
    if (centerBox(1) == P)
        {
        MTOOLS_ERROR("This case must be treated separately !");
        return v;
        }
    int nb = 0;
    _pathToReach(P, 7, v, nb);
    return v;
    }
*/








/**
* Compute the index of the next cut point for a given arm.
**/
/*
    bool Nextcut(int arm_index, const std::vector<iVec2>& tour, int index, int& cut_index, int& cut_target_angle)
        {
        //   MTOOLS_INSURE(tour[index] == pos());
        const int l = lenArm(arm_index);
        const int a = angle(arm_index);
        const iVec2 C = centerBox(arm_index);

        if ((a >= 0) && (a < 2 * l))
            {  // cut when entering upper half : y > y0
            const int y0 = (int)(C.Y() + l);
            for (int i = index; i < tour.size(); i++)
                {
                if (tour[i].Y() > y0)
                    { // found !
                    cut_index = i;
                    cut_target_angle = (tour[i].X() < C.X()) ? (0) : (2 * l);
                    return true;
                    }
                }
            cut_index = (int)tour.size();
            cut_target_angle = l;
            return false;
            }

        if ((a >= 2 * l) && (a < 4 * l))
            { // cut when entering left side : x < x0
            const int x0 = (int)(C.X() - l);
            for (int i = index; i < tour.size(); i++)
                {
                if (tour[i].X() < x0)
                    { // found !
                    cut_index = i;
                    cut_target_angle = (tour[i].Y() < C.Y()) ? (2 * l) : (4 * l);
                    return true;
                    }
                }
            cut_index = (int)tour.size();
            cut_target_angle = 3 * l;
            return false;
            }

        if ((a >= 4 * l) && (a < 6 * l))
            { // cut when entering lower half: y < y0
            const int y0 = (int)(C.Y() - l);
            for (int i = index; i < tour.size(); i++)
                {
                if (tour[i].Y() < y0)
                    { // found !
                    cut_index = i;
                    cut_target_angle = (tour[i].X() < C.X()) ? (6 * l) : (4 * l);
                    return true;
                    }
                }
            cut_index = (int)tour.size();
            cut_target_angle = 5 * l;
            return false;
            }

        if ((a >= 6 * l) && (a < 8 * l))
            { // cut when entering right side: x > x0
            const int x0 = (int)(C.X() + l);
            for (int i = index; i < tour.size(); i++)
                {
                if (tour[i].X() > x0)
                    { // found !
                    cut_index = i;
                    cut_target_angle = (tour[i].Y() < C.Y()) ? (8 * l) : (6 * l);
                    return true;
                    }
                }
            cut_index = (int)tour.size();
            cut_target_angle = 7 * l;
            return false;
            }

        return false;
        }

        */