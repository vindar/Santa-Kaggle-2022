#include "Rectify.h"







bool _rectok(std::vector<Arm>& vec, Arm e, int ind)
    {
    for (int i = ind + 1; i < (int)vec.size(); i++)
        {
        if (vec[i].pos() != (vec[i] + e).pos()) return false;
        }
    // ok, we can rectify !
    for (int i = ind + 1; i < (int)vec.size(); i++)
        {
        vec[i] = vec[i] + e;
        }
    return true;
    }


/**
* Try to insert a dir*1 for arm arm_index somewhere in the path
* without changing the pixel visited.
**/
bool _insertmove(std::vector<Arm>& vec, int dir, int arm_index)
    {
    for (int i = (int)vec.size() - 2; i >= 0; i--)
        {
        const Arm d = vec[i + 1] - vec[i];
        if (!d.isValidStep()) return false; // this is a gap, we stop. 
        // ok, d is a valid +1, 0, -1 step.
        const int ss = d.sign(arm_index);
        if (ss == -dir)
            { // we try to set the angle to 0 and then set another one from 0 to 1 
            Arm b = d;
            b.setAngle(arm_index, 0);
            for (int j = arm_index - 1; j >= 0; j--)
                {
                if (b.angle(j) == 0)
                    {
                    Arm c = b;
                    c.setAngle(j, 1);
                    if (_rectok(vec, c - d, i)) return true;
                    c.setAngle(j, -1);
                    if (_rectok(vec, c - d, i)) return true;
                    }
                }
            } 
        else if (ss == 0)        
            { // we set the angle to 1 and try to set another one to 0 
            Arm b = d;
            b.setAngle(arm_index, dir);
            for (int j = arm_index - 1; j >= 0; j--)
                {
                if (b.angle(j) != 0)
                    {
                    Arm c = b;
                    c.setAngle(j, 0);
                    if (_rectok(vec, c - d, i)) return true;
                    }
                }
            }
            // here ss = dir, nothing can be done. 
        }
    return false;
    }



/**
* Try to rectify a path to get closer to P at the end.
**/
int rectify(std::vector<Arm>& vec, iVec2 P)
    {
    int totmove = 0;
    Arm ada;
    ada.setZero();
    for (int arm_index = 7; arm_index >= 0; arm_index--)
        { // we are working with arm i, 
        Arm tip = vec.back() + ada; // make sure the arm with larger index contain the point

        bool err;
        auto R = tip.anglesToReach(P, arm_index, err); // find how much we should move arm_index
        MTOOLS_INSURE((err == false) || (arm_index == 0)); // should only fail for arm_index=0; 
        if (err)
            { // ok, special move, we do not deal with this here.
          //  cout << "SPECIAL NO RECTIFYIED\n";
            return totmove;
            }
        // find the shortest direction to move
        int n = (abs(R.first) < abs(R.second)) ? R.first : R.second;
        int dir = (n >= 0) ? 1 : -1; // direction
        n = abs(n); // number of steps
        // rectify this arm as much as possible
        while ((n > 0) && (_insertmove(vec, dir, arm_index)))
            {
            n--;
            totmove++;
            }
        // ok, all possible insertions were made
        ada.setAngle(arm_index, n * dir); // save the new distance for this arm. 
        }
    // rectification completed. 
    return totmove;
    }





/** end of file */
