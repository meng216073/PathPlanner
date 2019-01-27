//  dubinspath.cpp
//  Dubins path generator (implementation).

#define _USE_MATH_DEFINES
#include <cmath>

#include "dubinspath.h"
#include "util.h"

using namespace std;
using namespace dubins;

int DubinsPath::getDubinsWords(int dubinPathType, double alpha, double beta, double d, dubinsLength& outputs) const
{
  double sa = sin(alpha);
  double sb = sin(beta);
  double ca = cos(alpha);
  double cb = cos(beta);
  double c_ab = cos(alpha - beta);

  if (dubinPathType == LSL)
{
    double tmp0 = d+sa-sb;
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));

    if( p_squared < 0 ) 
    {
        return ERR_NOPATH;
    }

    double tmp1 = atan2( (cb-ca), tmp0 );
    double t = foldOverTwoPi(-alpha + tmp1 );
    double p = sqrt( p_squared );
    double q = foldOverTwoPi(beta - tmp1 );

    outputs.seg1 = t;
    outputs.seg2 = p;
    outputs.seg3 = q;

    return ERR_OK;
  }
  else if (dubinPathType == LSR)
  {
    double p_squared = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));

    if( p_squared < 0 ) 
    {
        return ERR_NOPATH;
    }

    double p    = sqrt( p_squared );
    double tmpLSR = atan2( (-ca-cb), (d+sa+sb) ) - atan2(-2.0, p);
    double t    = foldOverTwoPi(-alpha + tmpLSR);
    double q    = foldOverTwoPi( -foldOverTwoPi(beta) + tmpLSR );

    outputs.seg1 = t;
    outputs.seg2 = p;
    outputs.seg3 = q;

    return ERR_OK;
  }
  else if (dubinPathType == RSL)
  {
    double p_squared = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));

    if( p_squared < 0 ) 
    {
        return ERR_NOPATH;
    }

    double p    = sqrt( p_squared );
    double tmpRSL = atan2( (ca+cb), (d-sa-sb) ) - atan2(2.0, p);
    double t    = foldOverTwoPi(alpha - tmpRSL);
    double q    = foldOverTwoPi(beta - tmpRSL);

    outputs.seg1 = t;
    outputs.seg2 = p;
    outputs.seg3 = q;

    return ERR_OK;
  }
  else if (dubinPathType == RSR)
  {
    double tmp0 = d-sa+sb;
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));

    if( p_squared < 0 ) 
    {
        return ERR_NOPATH;
    }

    double tmpRSR = atan2( (ca-cb), tmp0 );
    double t = foldOverTwoPi( alpha - tmpRSR );
    double p = sqrt( p_squared );
    double q = foldOverTwoPi( -beta + tmpRSR );

    outputs.seg1 = t;
    outputs.seg2 = p;
    outputs.seg3 = q;

    return ERR_OK;
  }
  else if (dubinPathType == RLR)
  {
    double tmpRLR = (6.0 - d*d + 2*c_ab + 2*d*(-sa + sb)) / 8.0;

    if( fabs(tmpRLR) > 1 ) 
    {
        return ERR_NOPATH;
    }

    double p = foldOverTwoPi( 2*M_PI - acos( tmpRLR ) );
    double t = foldOverTwoPi(-alpha - atan2( ca-cb, d+sa-sb ) + p/2.0);
    double q = foldOverTwoPi(foldOverTwoPi(beta) - alpha - t + foldOverTwoPi(p));

    outputs.seg1 = t;
    outputs.seg2 = p;
    outputs.seg3 = q;

    return ERR_OK;
  }
  else if (dubinPathType == LRL)
  {
    double tmpLRL = (6.0 - d*d + 2*c_ab + 2*d*(sa - sb)) / 8.0;

    if( fabs(tmpLRL) > 1) 
    {
        return ERR_NOPATH;
    }

    double p = foldOverTwoPi( 2*M_PI - acos( tmpLRL ) );
    double t = foldOverTwoPi(alpha - atan2( ca-cb, d-sa+sb ) + foldOverTwoPi(p/2.0));
    double q = foldOverTwoPi(alpha - beta - t + foldOverTwoPi(p));

    outputs.seg1 = t;
    outputs.seg2 = p;
    outputs.seg3 = q;

    return ERR_OK;
  }
    return ERR_NOPATH;
}


int DubinsPath::getNextSegment(double t, const state& qi, state& qt, int pathType) const
{
  if(pathType != L_SEG && pathType != R_SEG && pathType != S_SEG)
  {
    return ERR_NOPATH;
  }

  if( pathType == L_SEG ) 
  {
    qt.x = qi.x + sin(qi.theta + t) - sin(qi.theta);
    qt.y = qi.y - cos(qi.theta + t) + cos(qi.theta);
    qt.theta = qi.theta + t;
    return ERR_OK;
  }
  else if( pathType == R_SEG ) 
  {
    qt.x = qi.x - sin(qi.theta - t) + sin(qi.theta);
    qt.y = qi.y + cos(qi.theta - t) - cos(qi.theta);
    qt.theta = qi.theta - t;
    return ERR_OK;
  }
  else if( pathType == S_SEG ) 
  {
    qt.x = qi.x + cos(qi.theta) * t;
    qt.y = qi.y + sin(qi.theta) * t;
    qt.theta = qi.theta;
    return ERR_OK;
  }
}


int DubinsPath::init(const state& startState, const state& goalState)
{
  // It doesn't make sense to have a negative or 0 turning radius
  if (turnRadius <= 0)
  {
    return ERR_BADRHO;
  }

  double deltaX = goalState.x - startState.x;
  double deltaY = goalState.y - startState.y;

  double dist2Goal = sqrt(deltaX * deltaX + deltaY * deltaY);
  double d = dist2Goal / turnRadius;

  double theta = atan2(deltaY, deltaX);
  double alpha = startState.theta - theta;
  double beta = goalState.theta - theta;

  this->startState.x = startState.x;
  this->startState.y = startState.y;
  this->startState.theta = startState.theta;

  this->turnRadius = turnRadius;
  normalize(alpha, beta, d);

  return ERR_OK;
}


int DubinsPath::normalize(double alpha, double beta, double d)
{
  double bestCost = INFINITY;
  double bestWord = -1;

  for( int i = 0; i < 6; i++ )
  {
      dubinsLength output;
      int err = getDubinsWords(i, alpha, beta, d, output);

      if(err == ERR_OK)
      {
          double cost = output.seg1 + output.seg2 + output.seg3;
          if(cost < bestCost)
          {
              bestWord = i;
              bestCost = cost;
              this->segLengths.seg1 = output.seg1;
              this->segLengths.seg2 = output.seg2;
              this->segLengths.seg3 = output.seg3;
              this->pathType = i;
          }
      }
  }

  if(bestWord == -1)
  {
    return ERR_NOPATH;
  }

  return ERR_OK;

}


int DubinsPath::pathSample(state& currentState, double t) const
{
    if( t < 0 || t >= this->getPathLength() ) 
    {
      // error, parameter out of bounds
      return ERR_PARAM;
    }

    // tprime is the normalised variant of the parameter t
    double tprime = t / this->turnRadius;

    // In order to take rho != 1 into account this function needs to be more complex
    // than it would be otherwise. The transformation is done in five stages.
    //
    // 1. translate the components of the initial configuration to the origin
    // 2. generate the target configuration
    // 3. transform the target configuration
    //      scale the target configuration
    //      translate the target configration back to the original starting point
    //      normalise the target configurations angular component

    // The translated initial configuration
    state qi = { 0, 0, this->startState.theta };

    // Generate the target configuration
    const int* types = DUBINDIRDATA[this->pathType];

    double p1 = this->segLengths.seg1;
    double p2 = this->segLengths.seg2;

    state q1; // end-of segment 1
    state q2; // end-of segment 2

    getNextSegment( p1, qi, q1, types[0] );
    getNextSegment( p2, q1, q2, types[1] );

    if( tprime < p1 )
    {
        getNextSegment( tprime, qi, currentState, types[0] );
    }
    else if( tprime < (p1+p2) )
    {
        getNextSegment( tprime-p1, q1, currentState,  types[1] );
    }
    else
    {
        getNextSegment( tprime-p1-p2, q2, currentState,  types[2] );
    }

    // scale the target configuration, translate back to the original starting point
    currentState.x = currentState.x * this->turnRadius + this->startState.x;
    currentState.y = currentState.y * this->turnRadius + this->startState.y;
    currentState.theta = foldOverTwoPi(currentState.theta);

    return 0;
}
