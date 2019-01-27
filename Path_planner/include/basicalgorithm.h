
//  basicalgorithm.h
//  Basic algorithms family (base class definition).

#ifndef BASICALGORITHM_H
#define BASICALGORITHM_H

#include "searchalgorithm.h"

namespace basicAlgo {

typedef enum {
  NOT_FOUND,
  FOUND
} pathResult;

};

class BasicAlgorithm : public SearchAlgorithm {

private:
  basicAlgo::pathResult pathRes;

protected:
  grid::heuristicType heuristic;

public:
  // Perform the path search
  virtual const Node* launchSearch(const SearchGrid& searchGrid) = 0;
  virtual ~BasicAlgorithm() {};

  // Set path result
  void setPathResult(const basicAlgo::pathResult& newPathRes) { pathRes = newPathRes; };

  // Get path result
  basicAlgo::pathResult getPathResult() const { return pathRes; };

  // Set heuristic
  void setHeuristic(grid::heuristicType hType) { heuristic = hType; }
};

#endif