//  smoother.cpp
//  Class used to smooth the path (implementation).

#include <cmath>
#include <iostream>

#include "smoother.h"
#include "util.h"

using namespace std;
using namespace smoother;

Vec2 Smoother::curvatureTerm(Vec2 xim1, Vec2 xi, Vec2 xip1) const
{
  Vec2 gradient;
  // the vectors between the nodes
  Vec2 deltaXi = xi - xim1;
  Vec2 deltaXip1 = xip1 - xi;
  // orthogonal complements vector
  Vec2 p1, p2;

  // the distance of the vectors
  double absDeltaXi = deltaXi.norm();
  double absDeltaXip1 = deltaXip1.norm();

  // ensure that the absolute values are not null
  if (absDeltaXi > 0 && absDeltaXip1 > 0)
  {
    double deltaPhi = acos(clamp(deltaXi.dot(deltaXip1) / (absDeltaXi * absDeltaXip1), -1, 1));

    // curvature of the path
    double curvature = deltaPhi / absDeltaXi;

    // if the curvature is smaller then the maximum do nothing
    if (curvature <= maxCurvature) 
    {
      return gradient; // = Vec2(0, 0)
    }
    else
    {
      double absDeltaXiInv = 1 / absDeltaXi;
      double PDeltaPhi_PcosDeltaPhi = -1 / sqrt(1 - pow(cos(deltaPhi), 2));
      double u = -absDeltaXiInv * PDeltaPhi_PcosDeltaPhi;

      // calculate the p1 and p2 terms
      p1 = xi.ort(-xip1) / (absDeltaXi * absDeltaXip1);
      p2 = -xip1.ort(xi) / (absDeltaXi * absDeltaXip1);

      // calculate the last terms
      double s = deltaPhi / (absDeltaXi * absDeltaXi);
      Vec2 ones(1, 1);
      Vec2 ki = (-p1 - p2) * u - (ones * s);
      Vec2 kim1 = p2 * u - (ones * s);
      Vec2 kip1 = p1 * u;

      // calculate the gradient
      gradient = (kim1 * 0.25 + ki * 0.5 + kip1 * 0.25) * curvatureWeight;

      // if NAN values in the gradient vector return 0
      if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
      {
        return Vec2(); // = Vec2(0, 0)
      }
      else
      {
        return gradient;
      }
    }
  }
  else
  {
    return Vec2(); // = Vec2(0, 0)
  }
}


Vec2 Smoother::smoothnessTerm(Vec2 xim2, Vec2 xim1, Vec2 xi, Vec2 xip1, Vec2 xip2) const
{
  return (xim2 - xim1 * 4 + xi * 6 - xip1 * 4 + xip2) * smoothnessWeight;
}


deque<state> Smoother::smooth(const deque<state>& path) const
{
  cout << endl << "Smoother initiated.." << endl;

  // current number of iterations of the gradient descent smoother
  int iterations = 0;

  int numberOfWaypoints = path.size();
  double totalWeight = smoothnessWeight + curvatureWeight;

  deque<state> newPath = path;

  // gradient descent
  while (iterations < maxIterations)
  {
   // get the third waypoint first, skipping the last one
   for (int i = 2; i < numberOfWaypoints - 3; ++i) 
   {
     Vec2 xim2(newPath[i - 2].x, newPath[i - 2].y);
     Vec2 xim1(newPath[i - 1].x, newPath[i - 1].y);
     Vec2 xi(newPath[i].x, newPath[i].y);
     Vec2 xip1(newPath[i + 1].x, newPath[i + 1].y);
     Vec2 xip2(newPath[i + 2].x, newPath[i + 2].y);
     Vec2 correction;

     correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
     correction = correction - curvatureTerm(xim1, xi, xip1);

     xi = xi + (correction/totalWeight) * alpha;
     newPath[i].x = xi.x();
     newPath[i].y = xi.y();
     Vec2 deltaXi = xi - xim1;
     newPath[i - 1].theta = foldOverTwoPi( atan2(deltaXi.y(), deltaXi.x()) );
   }

   iterations++;
  }

  cout << "Path successfully smoothed" << endl;

  return newPath;
}

deque<state> Smoother::getPathFromGoalNode(const Node* goalNode) const
{
  deque<state> path;
  int i = 0;
  int pos = goalNode->pos();

  while(goalNode != nullptr)
  {
    if(i < pos && i % 3 == 0)
    {
      path.push_front( state{ goalNode->x(), goalNode->y(), goalNode->theta() } );
    }
    else if(i >= pos)
    {
      path.push_front( state{ goalNode->x(), goalNode->y(), goalNode->theta() } );
    }

    i++;
    goalNode = goalNode->pred();
  }

  return path;
}
