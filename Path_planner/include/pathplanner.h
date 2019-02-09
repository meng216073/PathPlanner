
//  pathplanner.h
//  Main class (definition).

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <memory>
#include <vector>
#include <chrono>

#include "smoother.h"
#include "searchgrid.h"
#include "hybridalgorithm.h"

namespace planner {

// Error codes
const int ERR_OK = 0;             // No error
const int ERR_ALGO = 1;           // Search algorithm not initialized
const int ERR_GRID = 2;           // Search grid not created

const int ERR_POS_EQUAL = 4;      // Start = Goal
const int ERR_START_POS = 5;      // Start outside the map
const int ERR_GOAL_POS = 6;       // Goal outside the map
const int ERR_RESOLUTION = 7;     // Resolution < 0
const int ERR_LIMITS_X = 8;       // Map length < 0
const int ERR_LIMITS_Y = 9;       // Map width < 0
const int ERR_LENGTH = 10;        // Length < 0
const int ERR_WIDTH = 11;         // Width < 0

//Output files
const std::string OUTPUTS_DIR = "outputs";
const std::string WAYPOINTS_FILE = OUTPUTS_DIR + "/" + "waypoints.csv";
const std::string OBSTACLES_FILE = OUTPUTS_DIR + "/" + "obstacles.csv";
const std::string MAP_FILE = OUTPUTS_DIR + "/" + "map.csv";
const std::string WAYPOINTS_GPS_FILE = OUTPUTS_DIR + "/" + "waypoints_gps.csv";
const std::string OBSTACLES_GPS_FILE = OUTPUTS_DIR + "/" + "obstacles_gps.csv";
const std::string OBSTACLES_CARTESIAN_FILE = OUTPUTS_DIR + "/" + "obstacles_cartesian.csv";
};

template <class T>
class PathPlanner {

private:
  // Attributes
  std::unique_ptr<Smoother> smoother;
  std::unique_ptr<SearchGrid> searchGrid;
  T* searchAlgorithm;

  // Final smoothed path
  std::deque<state> finalPath;

  // Check input data (rectangles)
  int checkInputs(state& IstartState, state& IgoalState, std::pair<double,double> gridLimitsX, std::pair<double,double> gridLimitsY, double resolution, double safetyDist) const;

  // Check obstacles (rectangles)
  int checkObstacles(std::vector<obstacle>& IobstaclesArray, std::pair<double,double> gridLimitsX, std::pair<double,double> gridLimitsY) const;

  // Check obstacles (polygons)
  void checkObstacles(std::vector< std::vector<point> >& IobstaclesArray, std::pair<double,double> gridLimitsX, std::pair<double,double> gridLimitsY) const;

public:
  // Constructors (rectangles)
  PathPlanner(state& IstartState, state& IgoalState, std::vector<obstacle>& IobstaclesArray,
              double gridLengthX, double gridLengthY, double resolution, double safetyDist = grid::DEFAULT_SAFETY_DIST);

  // Constructors (polygons)
  PathPlanner(state& IstartState, state& IgoalState, std::vector< std::vector<point> >& IobstaclesArray,
              double gridLengthX, double gridLengthY, double resolution, double safetyDist = grid::DEFAULT_SAFETY_DIST);

  // Load obstacles
  void loadObstacles(const std::vector< std::vector<point> >& obstaclesArray) { searchGrid->loadObstacles(obstaclesArray); }

  // Getters
  std::vector< std::vector<point> > getObstaclesVertices() const { return searchGrid->getListVertices(); }
  T& getSearchAlgorithm() const { return *searchAlgorithm; }
  cell** getGrid() const { return searchGrid->getGrid(); }

  // Setters
  void setSearchAlgorithm(const std::string& name);
  int setSearchAlgorithmHeuristic(grid::heuristicType hType);
  int setListObstacles(std::vector<obstacle>& newListObstacles);
  int setListObstacles(std::vector< std::vector<point> >& newListObstacles);
  int setDist2Obstacles(double dist);
  void setGrid(cell** grid) { searchGrid->setGrid(grid); }
  void setStartState(const state& start) { searchGrid->setStartNode(start); }
  void setGoalState(const state& goal) { searchGrid->setGoalNode(goal); }

  // Execute the path planning
  int makePlan();
  int makeNextPlan(const state& startState, const state& goalState);

  // Compute altitude
  void setWaypointsAltitude(std::deque<state>& path) const;

  // Reduce number of waypoints
  void reduceNbWaypoints(std::deque<state>& path) const;


  // Return the final smoothed path
  const std::deque<state> getPath() const { return finalPath; }

  // Return the path result
  int getPathResult() const { return searchAlgorithm->getPathResult(); };

  // Delete path from goalNode
  void deletePath(const Node* goalNode) const;

  // Write outputs to .csv files
  void saveOutputs(double execTime);
};

#endif
