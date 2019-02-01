
//  searchgrid.h
//  Search grid containing the environment map (definition).

#ifndef SEARCHGRID_H
#define SEARCHGRID_H

#include <vector>
#include "node.h"

namespace grid {

// Error codes
const int ERR_INDX = -1;   // Index out of range
const int ERR_OK = 0;      // No error

const int NB_VERTICES_RECT = 5;  // Number of vertices in a rectangle (last one to close it)

const double DEFAULT_SAFETY_DIST = 2; // 2 meters

// Heuristics (important order!) => OCTILE used by default
typedef enum {
  OCTILE,
  EUCLIDEAN,
  MANHATTAN
} heuristicType;

};

typedef enum {
  freeC = 0,
  occupied = 1,
  start = 2,
  goal = 3,
  path = 4
} cell;

// (x,y) is the center of the rectangle
typedef struct {
  double x;
  double y;
  double theta;
  double length;
  double width;
} obstacle;

typedef struct {
  double x;
  double y;
  double z;
} point;

typedef struct {
	double longitude;
	double latitude;
	double altitude;
} coordinates;

typedef struct {
	double longitude;
	double latitude;
	double radius;
	double height;
} coordinatesObst;


class SearchGrid {

private:
  // Attributes
  Node* startNode;
  Node* goalNode;
  std::pair<double,double> gridLimitsX;
  std::pair<double,double> gridLimitsY;
  double resolution;
  int nbRows;
  int nbCols;
  int nbNodes;
  cell** grid; // 2D grid
  std::vector<point> gridCoordinates;
  double safetyDist;
  std::vector< std::vector<point> > listObstacles;

  // Methods
  // Get vertices of a rectangle obstacle
  std::vector<point> getVerticesObstacle(const obstacle& obstacle) const;

  // Extend obstacles to respect safety distance
  void extendObstacle(std::vector<point>& vertice, double extentionDist);

  // Extend a corner of an obstacle
  void extendCorner(double a, double b, double c, double d, double e, double f, point& corner, double extentionDist);

  // Get the intersection of two lines
  bool lineIntersection(double Ax, double Ay, double Bx, double By, double Cx, double Cy, double Dx, double Dy, point& intersection);

  // Test whether a polygon (an obstacle) is traced clockwise or counterclockwise
  bool isPolygonClockWise(const std::vector<point>& vertices) const;

  // Test whether a point is inside a polygon (used for obstacles)
  bool pointInPolygon(const std::vector<point>& vertices, const point& testPoint) const;

  // Find distance  from a point to a polygon (used for obstacles)
  double pointDistPolygon(const std::vector<point>& vertices, const point& testPoint) const;

public:
  // Constructor
  SearchGrid(const state& IstartState, const state& IgoalState, std::pair<double,double> IgridLimitsX, std::pair<double,double> IgridLimitsY, double Iresolution);

  // Destructor
  ~SearchGrid();

  // Getters
  const Node* getStartNode() const { return startNode; }
  const Node* getGoalNode() const { return goalNode; }
  const state getStartState() const { return state{ startNode->x(), startNode->y(), startNode->theta(), startNode->z() }; }
  const state getGoalState() const { return state{ goalNode->x(), goalNode->y(), goalNode->theta(), goalNode->z() }; }
  std::pair<double,double> getGridLimitsX() const { return gridLimitsX; }
  std::pair<double,double> getGridLimitsY() const { return gridLimitsY; }
  double getResolution() const { return resolution; }
  int getNbRows() const { return nbRows; }
  int getNbCols() const { return nbCols; }
  int getNbNodes() const { return nbNodes; }
  std::vector< std::vector<point> > getListVertices() const { return listObstacles; }
  double getSafetyDist() const { return safetyDist; }
  cell** getGrid() const { return grid; }

  // Setters
  void setSafetyDist(double dist) { safetyDist = dist; }
  void setGrid(cell** newGrid) { grid = newGrid; }
  void setStartNode(const state& start) { startNode->setX(start.x); startNode->setY(start.y); startNode->setTheta(start.theta); };
  void setGoalNode(const state& goal) { goalNode->setX(goal.x); goalNode->setY(goal.y); goalNode->setTheta(goal.theta); };

  // Check if a node is far enough from an obstacle
  bool nodeFarFromObstacle(const Node& node) const;

  // Get state of a cell from its index
  cell getCellFromIndex(int index) const;

  // Set state of a cell from its index
  void setCellFromIndex(int index);

  // Convert a node/point (x, y ...) to an index in the grid
  int convertNode2Indx(const Node& node) const;
  int convertPoint2Indx(const point& pt) const;

  // Fill in the grid
  int createWorkspace();

  // Fill in the grid with the obstacles (rectangles)
  void loadObstacles(const std::vector<obstacle>& obstaclesArray);

  // Fill in the grid with the obstacles (polygon vertices)
  void loadObstacles(std::vector< std::vector<point> > newListObstacles);

  // Check if the node is reachable
  bool isNodeTraversable(const Node& node) const;

  // Check if the goal is reachable from a given node
  bool isGoalClose(const Node& node, double triggerDistance) const;

  // Get the distance from a node to the goal
  double getDistance2Goal(const Node& node) const;

  // Check if the node is within the grid or free space
  bool isNodeWithinGrid(const Node& node) const;

  // Check if the point is within the grid or free space
  bool isPointWithinGrid(const point& pt) const;

  // Check if the index is within the grid or free space
  bool isIndexWithinGrid(int index) const;

  // Calculate potential at this node
  double calculatePotential(const Node& node) const;

  // Calculate heuristic at this node
  double calculateHeuristic(const Node& node, grid::heuristicType hType) const;
};

#endif