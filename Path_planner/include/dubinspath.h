//  dubinspath.h
//  Dubins path generator (definition).

#ifndef DUBINSPATH_H
#define DUBINSPATH_H

#include "node.h"

namespace dubins {

// This is the minimum distance to the goal that triggers the dubins path shoot call
const double TRIGGERDISTANCE = 2;

// The maximum number of nodes in a dubins path
const unsigned int MAXNODESINPATH  = TRIGGERDISTANCE * 500;

// Maximum dubins path length
const double MAXPATHLENGTH = TRIGGERDISTANCE * 2;

// All 6 different type of a dubin path
const unsigned int LSL = 0;
const unsigned int LSR = 1;
const unsigned int RSL = 2;
const unsigned int RSR = 3;
const unsigned int RLR = 4;
const unsigned int LRL = 5;

// The three general path types
const unsigned int L_SEG = 0;
const unsigned int S_SEG = 1;
const unsigned int R_SEG = 2;

// Error codes
const unsigned int ERR_OK        = 0;   // No error
const unsigned int ERR_COCONFIGS = 1;   // Colocated configurations
const unsigned int ERR_PARAM     = 2;   // Path parameterisation error
const unsigned int ERR_BADRHO    = 3;   // The radius is invalid
const unsigned int ERR_NOPATH    = 4;   // No connection between configurations with this word

// The segment types for each of the path types
const int DUBINDIRDATA[6][3] = {
  { L_SEG, S_SEG, L_SEG },
  { L_SEG, S_SEG, R_SEG },
  { R_SEG, S_SEG, L_SEG },
  { R_SEG, S_SEG, R_SEG },
  { R_SEG, L_SEG, R_SEG },
  { L_SEG, R_SEG, L_SEG }
};

};

typedef struct {
  double seg1;
  double seg2;
  double seg3;
} dubinsLength;

class DubinsPath {

private:
  // Attributes
  state startState;        // The initial state
  dubinsLength segLengths; // The lengths of the three dubin segments
  double turnRadius;       // The turning radius
  int pathType;            // LSL, LSR, RSL, RSR, RLR or LRL

  // Methods
  // Get the best word (LSL, LSR...) in terms of cost (after init)
  int normalize(double alpha, double beta, double d);

  // Compute LSL, LSR, RSL, RSR, RLR or LRL
  int getDubinsWords(int dubinPathType, double alpha, double beta, double d, dubinsLength& outputs) const;

  // Extract a subpath from a path
  int getNextSegment(double t, const state& qi, state& qt, int pathType) const; 

public:
  // Constructor
  DubinsPath() : startState{0}, segLengths{0}, turnRadius(0), pathType(-1) {}

  // Getters
  const state getStartState() const { return startState; }
  const dubinsLength getSegLengths() const { return segLengths; }
  double getPathLength() const { return (segLengths.seg1 + segLengths.seg2 + segLengths.seg3) * turnRadius; }
  int getPathType() const { return pathType; }
  double getTurnRadius() const { return turnRadius; }

  // Setters
  void setTurningRadius(double newTurnRadius) { turnRadius = newTurnRadius; }

  // Initialisation of the dubins path with a specified minimum turning radius
  int init(const state& startState, const state& goalState);

  // Generate the target configuration from an initial configuration
  int pathSample(state& currentState, double t) const;
};

#endif
