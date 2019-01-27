//  smoother.h
//  Class used to smooth the path (definition).

#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <deque>
#include "Vec2.h"
#include "node.h"

namespace smoother {

  // The maximum iterations for the gradient smoother
  const int maxIterations = 500;

  const double minTurnRadius = 0.02;
  const double maxCurvature = 1.0 / (minTurnRadius * 1.1);
  const double smoothnessWeight = 0.2;
  const double curvatureWeight = 0.2;
  const double alpha = 0.1; // falloff rate for the voronoi field
};

class Smoother {

public:
  // Curvature term
  Vec2 curvatureTerm(Vec2 xim1, Vec2 xi, Vec2 xip1) const;

  // Smoothness term
  Vec2 smoothnessTerm(Vec2 xim2, Vec2 xim1, Vec2 xi, Vec2 xip1, Vec2 xip2) const;

  // Smooth the path
  std::deque<state> smooth(const std::deque<state>& path) const; 

  // Retrieve the path from the goal node
  std::deque<state> getPathFromGoalNode(const Node* goalNode) const;
};

#endif