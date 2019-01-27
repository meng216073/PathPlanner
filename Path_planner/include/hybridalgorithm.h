
//  hybridalgorithm.h
//  Hybrid algorithms family (base class definition).

#ifndef HYBRIDALGORITHM_H
#define HYBRIDALGORITHM_H

#include "dubinspath.h"
#include "basicalgorithm.h"

namespace hybridAlgo {

typedef enum {
  NOT_FOUND,
  FOUND,
  DUBINS_CLOSE
} pathResult;

};

class HybridAlgorithm : public BasicAlgorithm {

private:
  hybridAlgo::pathResult pathRes;

protected:
  // Attributes
  DubinsPath dubins;
  bool isDubinsHeuristic = true;
  double velocity; // Velocity
  double omegaMax; // Angular velocity

public:
  // Perform the path search
  virtual const Node* launchSearch(const SearchGrid& searchGrid) = 0;
  virtual ~HybridAlgorithm() {};

  // Set the drone velocity
  void setVelocity(double V) { velocity = V; };

  // Set the drone velocity
  void setAngularVelocity(double W) { omegaMax = W; };

  // Set dubins turning radius
  void setDubinsTurnRadius(double turnRadius) { dubins.setTurningRadius(turnRadius); };

  // Set dubins boolean
  void doNotUseDubins() { isDubinsHeuristic = false; };

  // Set path result
  void setPathResult(const hybridAlgo::pathResult& newPathRes) { pathRes = newPathRes; };

  // Get path result
  hybridAlgo::pathResult getPathResult() const { return pathRes; };
};

#endif