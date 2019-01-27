
//  astarhybrid.h
//  Hybrid A* search algorithm (definition).

#ifndef ASTARHYBRID_H
#define ASTARHYBRID_H

#include "hybridalgorithm.h"

namespace astarHybrid {

// Number of possible directions
const int numberOfAnglesToDrive = 3;

};

class AStarHybrid  : public HybridAlgorithm {

private:
  // Dubins shoot used to get a dubins path
  Node* shoot(const SearchGrid& searchGrid, const Node& startNode, const Node& goalNode);

  // Calculate the heuristic value from a given node
  void findHeuristic(const SearchGrid& searchGrid, Node& startNode, const Node& goalNode);

  // Set the turning radius of the dubins path
  void setDubinsTurnRadius();

public:
  // Perform the path search
  const Node* launchSearch(const SearchGrid& searchGrid);
  ~AStarHybrid();

  // Get an instance
  static AStarHybrid* create();
};

#endif