#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "slip.h"
#include <ros/ros.h>
#include "gki_utils/misc.h"

double detectSlip(const TransValues &msg)
{
   double leftDiff = fabs(msg.transVelLeftFront - msg.transVelLeftRear);
   double rightDiff = fabs(msg.transVelRightFront - msg.transVelRightRear);
   //double frontDiff = fabs(msg.transVelLeftFront - msg.transVelRightFront);
   //double rearDiff = fabs(msg.transVelLeftRear - msg.transVelRightRear);

   bool turn = false;
   // detect turn:
   double maxSideDiff = MAX(leftDiff, rightDiff);
   //double minLengthDiff = MIN(frontDiff, rearDiff);
   //if(maxSideDiff < minLengthDiff && fabs(maxSideDiff - minLengthDiff) > 350)
     // turn = true;
   if(msg.transVelRightRear * msg.transVelLeftRear < 0 && msg.transVelRightFront * msg.transVelLeftFront < 0)  // tv entgegeng. vorzeichen -> turn
      turn = true;

   double slip = 0.0;
   if(turn) {
      slip = 0.5 * StraightUp(maxSideDiff, 0, 2500);
   } else {
      slip = StraightUp(maxSideDiff, 0, 500);
   }
   //slip = slip*slip;
/*
   rescue_test_dummy_message dummy;
   dummy.i1 = turn ? 1 : 0;
   dummy.i2 = int(slip * 100000);
   dummy.d1 = leftDiff;
   dummy.d2 = rightDiff;
   dummy.d3 = frontDiff;
   dummy.d4 = rearDiff;

   dummy.robot.ts = msg.robot.ts;
   dummy.robot.id = msg.robot.id;

   ComPublishToRobot( RESCUE_TEST_DUMMY_NAME, &dummy);
   printf("leftDiff=%lf rightDiff=%lf frontDiff=%lf, rearDiff=%lf  Turn: %d, Slip: %.2f\n", leftDiff, rightDiff, frontDiff, rearDiff, turn, slip);
*/
   return slip;
}

