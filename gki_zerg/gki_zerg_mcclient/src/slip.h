#ifndef SLIP_H
#define SLIP_H

struct TransValues
{
  int transVelLeftFront;               //mmPerSec
  int transVelLeftRear;                //mmPerSec
  int transVelRightFront;              //mmPerSec
  int transVelRightRear;               //mmPerSec
};


double detectSlip(const TransValues &msg);

#endif

