//  searchgrid.cpp
//  Search grid containing the environment map (implementation).

#include <cmath>
#include <iostream>
#include <algorithm> 

#include "searchgrid.h"
#include "util.h"

using namespace std;
using namespace grid;

SearchGrid::SearchGrid(const state& IstartState, const state& IgoalState, std::pair<double,double> IgridLimitsX, std::pair<double,double> IgridLimitsY, double Iresolution)
: startNode(new Node(IstartState.x, IstartState.y, IstartState.theta, IstartState.z)), goalNode(new Node(IgoalState.x, IgoalState.y, IgoalState.theta, IgoalState.z)),
  gridLimitsX(IgridLimitsX), gridLimitsY(IgridLimitsY), resolution(Iresolution)
{
  nbCols = ((gridLimitsX.second - gridLimitsX.first) / resolution) + 1;
  nbRows = ((gridLimitsY.second - gridLimitsY.first) / resolution) + 1;
  nbNodes = nbCols * nbRows;

  // grid init
  grid = new cell*[nbRows];

  for(int i = 0; i < nbRows; ++i)
  {
    grid[i] = new cell[nbCols];

    for(int j = 0; j < nbCols; ++j)
    {
      grid[i][j] = freeC;
    }
  }

  vector<double> linX = linspace(gridLimitsX.first, gridLimitsX.second, nbCols);
  vector<double> linY = linspace(gridLimitsY.first, gridLimitsY.second, nbRows);

  for(int i = 0; i < linX.size(); ++i)
  {
    for(int j = 0; j < linY.size(); ++j)
    {
      gridCoordinates.push_back( point{linX[i], linY[j]} );
    }
  }

  safetyDist = DEFAULT_SAFETY_DIST;
}

SearchGrid::~SearchGrid()
{
  delete startNode;
  delete goalNode;

  for(int i = 0; i < nbRows; ++i)
  {
    delete grid[i];
  }

  delete grid;
}

void SearchGrid::setCellFromIndex(int index)
{
  int j = index / nbRows;
  int i = index - (j * nbRows);

  if( grid[i][j] != start && grid[i][j] != goal )
  {
    grid[i][j] = path;
  }
}

cell SearchGrid::getCellFromIndex(int index) const
{
  int j = index / nbRows;
  int i = index - (j * nbRows);
  return grid[i][j];
}


int SearchGrid::convertNode2Indx(const Node& node) const
{
  double x = node.x();
  double y = node.y();

  if(x < gridLimitsX.first || x > gridLimitsX.second || y < gridLimitsY.first || y > gridLimitsY.second)
  {
    return ERR_INDX;
  }

  // Find how many columns there are between current point and the first column from left on the grid
  int nodeCols = fabs(x - gridLimitsX.first) / resolution;

  // Calculate the total number of elements in columns up to the current one
  // if the node lies on the first column there exist no previous column
  int nbElemXX = nodeCols * nbRows;

  // Calculate the number of elements from the beginning of the current column up to the current point
  int nbElemXY = (fabs(y - gridLimitsY.first) / resolution) + 1;

  return nbElemXX + nbElemXY - 1;
}


int SearchGrid::convertPoint2Indx(const point& pt) const
{
  double x = pt.x;
  double y = pt.y;

  if(x < gridLimitsX.first || x > gridLimitsX.second || y < gridLimitsY.first || y > gridLimitsY.second)
  {
    return ERR_INDX;
  }

  // Find how many columns there are between current point and the first column from left on the grid
  int nodeCols = fabs(x - gridLimitsX.first) / resolution;

  // Calculate the total number of elements in columns up to the current one
  // if the node lies on the first column there exist no previous column
  int nbElemXX = nodeCols * nbRows;

  // Calculate the number of elements from the beginning of the current column up to the current point
  int nbElemXY = (fabs(y - gridLimitsY.first) / resolution) + 1;

  return nbElemXX + nbElemXY - 1;
}


int SearchGrid::createWorkspace()
{
  // start and goal points
  int startIndx = convertNode2Indx(*startNode);
  if(startIndx == ERR_INDX)
  {
    return ERR_INDX;
  }

  int j = startIndx / nbRows;
  int i = startIndx - (j * nbRows);
  grid[i][j] = start;

  int goalIndx = convertNode2Indx(*goalNode);
  if(goalIndx == ERR_INDX)
  {
    return ERR_INDX;
  }

  j = goalIndx / nbRows;
  i = goalIndx - (j * nbRows);
  grid[i][j] = goal;

  return ERR_OK;
}


vector<point> SearchGrid::getVerticesObstacle(const obstacle& obstacle) const
{
  double x = obstacle.x;
  double y = obstacle.y;
  double theta = obstacle.theta;
  double L = obstacle.length;
  double W = obstacle.width;

  const int rotationMatrixRow = 2;
  const int rotationMatrixCol = 2;
  double rotationMatrix[rotationMatrixRow][rotationMatrixCol] = {
    {cos(theta), sin(theta)},
    {-sin(theta), cos(theta)}
  };

  const int verticesMatrixRow = NB_VERTICES_RECT;
  const int verticesMatrixCol = 2;
  double initialVertices[verticesMatrixRow][verticesMatrixCol] = {
    {-L/2, -W/2},
    {L/2, -W/2},
    {L/2,  W/2},
    {-L/2,  W/2},
    {-L/2, -W/2}
  };

  // Multiplication vertices * rotationMatrix
  double verticeRotation[verticesMatrixRow][rotationMatrixCol] = {0};
  for(int i = 0; i < verticesMatrixRow; ++i)
  {
    for(int j = 0; j < rotationMatrixCol; ++j)
    {
      for(int k = 0; k < verticesMatrixCol; ++k)
      {
        verticeRotation[i][j] += initialVertices[i][k] * rotationMatrix[k][j];
      }
    }
  }

  // Add coordinates
  vector<point> finalVertices;
  finalVertices.reserve(NB_VERTICES_RECT);
  for(int i = 0; i < verticesMatrixRow; ++i)
  {
    verticeRotation[i][0] += x;
    verticeRotation[i][1] += y;

    finalVertices.push_back( point{verticeRotation[i][0], verticeRotation[i][1]} );
  }

  return finalVertices;
}

bool SearchGrid::isPolygonClockWise(const std::vector<point>& vertices) const
{
  double sum = 0;
  int N = vertices.size();

  for(int i = 0; i < N; ++i)
  {
      sum += (vertices[(i+1) % N].x - vertices[i].x) * (vertices[(i+1) % N].y + vertices[i].y);
  }

  return (sum > 0);
}


bool SearchGrid::pointInPolygon(const vector<point>& vertices, const point& testPoint) const
{
  int i, j;
  bool c = false;

  for (i = 0, j = vertices.size()-1; i < vertices.size(); j = i++)
  {
    if ( ((vertices[i].y > testPoint.y) != (vertices[j].y > testPoint.y)) &&
         (testPoint.x < (vertices[j].x-vertices[i].x) * (testPoint.y-vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x) ) 
    {
      c = !c;
    }
  }
  return c;
}


double SearchGrid::pointDistPolygon(const vector<point>& vertices, const point& testPoint) const
{
  // Point inside the polygon
  if(pointInPolygon(vertices, testPoint)) {
    return -1.0;
  }

  double dist = 10000;

  for(int i = 0; i < vertices.size(); ++i)
  {
    int previousIndex = i - 1;
    if(previousIndex < 0)
    {
        previousIndex = vertices.size() - 2;
    }

    point currentPoint = vertices[i];
    point previousPoint = vertices[previousIndex];

    double deltaX = currentPoint.x - previousPoint.x;
    double deltaY = currentPoint.y - previousPoint.y;

    double u = ((testPoint.x - previousPoint.x) * deltaX + (testPoint.y - previousPoint.y) * deltaY) / (deltaX * deltaX + deltaY * deltaY);

    point closestPointOnLine;

    if (u < 0)
    {
	    closestPointOnLine = previousPoint;
  	}
    else if (u > 1)
    {
  	  closestPointOnLine = currentPoint;
  	}
    else
    {
      closestPointOnLine = point{ previousPoint.x + u * deltaX, previousPoint.y + u * deltaY };
  	}

    point d;
    d.x = testPoint.x - closestPointOnLine.x;
    d.y = testPoint.y - closestPointOnLine.y;

    double segmentDistance = d.x * d.x + d.y * d.y;

    if(segmentDistance < dist)
    {
        dist = segmentDistance;
    }
  }

  return dist; // sqrt(dist)
}

bool SearchGrid::lineIntersection(double Ax, double Ay, double Bx, double By, double Cx, double Cy, double Dx, double Dy, point& intersection) 
{
  double distAB, theCos, theSin, newX, ABpos;

  //  Fail if either line is undefined
  if (Ax == Bx && Ay == By || Cx == Dx && Cy == Dy) 
    return false;

  // (1) Translate the system so that point A is on the origin
  Bx -= Ax; 
  By -= Ay;
  Cx -= Ax; 
  Cy -= Ay;
  Dx -= Ax; 
  Dy -= Ay;

  //  Discover the length of segment A-B
  distAB = sqrt(Bx * Bx + By * By);

  // (2) Rotate the system so that point B is on the positive X axis
  theCos = Bx / distAB;
  theSin = By / distAB;
  newX = Cx * theCos + Cy * theSin;
  Cy = Cy * theCos - Cx * theSin; 
  Cx = newX;
  newX = Dx * theCos + Dy * theSin;
  Dy = Dy * theCos - Dx * theSin; 
  Dx = newX;

  // Fail if the lines are parallel
  if (Cy == Dy) 
    return false;

  // (3) Discover the position of the intersection point along line A-B
  ABpos = Dx + (Cx - Dx) * Dy / (Dy - Cy);

  // (4) Apply the discovered position to line A-B in the original coordinate system
  intersection.x = Ax + ABpos * theCos;
  intersection.y = Ay + ABpos * theSin;

  // Success
  return true; 
}

void SearchGrid::extendCorner(double a, double b, double c, double d, double e, double f, point& corner, double extentionDist) 
{     
  double  c1 = c, d1 = d, c2 = c, d2 = d, dx1, dy1, dist1, dx2, dy2, dist2;
  point extendedPoint;

  // Calculate length of line segments
  dx1 = c - a; 
  dy1 = d - b; 
  dist1 = sqrt(dx1 * dx1 + dy1 * dy1);
  dx2 = e - c; 
  dy2 = f - d; 
  dist2 = sqrt(dx2 * dx2 + dy2 * dy2);

  // Exit if either segment is zero-length
  if (dist1 == 0 || dist2 == 0) 
    return;

  // Extend each of the two line segments
  extendedPoint.x = dy1 / dist1 * extentionDist; 
  a += extendedPoint.x; 
  c1 += extendedPoint.x;

  extendedPoint.y = -dx1 / dist1 * extentionDist; 
  b += extendedPoint.y; 
  d1 += extendedPoint.y;

  extendedPoint.x = dy2 / dist2 * extentionDist; 
  e += extendedPoint.x; 
  c2 += extendedPoint.x;

  extendedPoint.y = -dx2 / dist2 * extentionDist; 
  f += extendedPoint.y; 
  d2 += extendedPoint.y;

  // If extended segments connect perfectly, return the connection point
  if (c1 == c2 && d1 == d2) 
  {
    corner.x = c1; 
    corner.y = d1; 
    return; 
  }

  // Return the intersection point of the two extended segments (if any)
  if (lineIntersection(a, b, c1, d1, c2, d2, e, f, extendedPoint)) 
  {
    corner.x = extendedPoint.x; 
    corner.y = extendedPoint.y; 
  }
}

void SearchGrid::extendObstacle(vector<point>& vertice, double extentionDist) 
{
  double startX = vertice[0].x, startY = vertice[0].y, a, b, c, d, e, f;
  int i;

  // Polygon must have at least three corners to be extended
  if (vertice.size() < 3) 
    return;

  // Extend the polygon.
  c = vertice[vertice.size() - 1].x; 
  d = vertice[vertice.size() - 1].y; 
  e = vertice[0].x; 
  f = vertice[0].y;

  for (i = 0; i < vertice.size() - 1; i++) 
  {
    a = c; 
    b = d; 
    c = e; 
    d = f; 
    e = vertice[i+1].x; 
    f = vertice[i+1].y;
    extendCorner(a, b, c, d, e, f, vertice[i], extentionDist); 
  }

  extendCorner(c, d, e, f, startX, startY, vertice[i], extentionDist); 
}


void SearchGrid::loadObstacles(const vector<obstacle>& obstaclesArray)
{
  cout << endl << "Loading obstacles.." << endl;

  int indx, i, j;

  listObstacles.reserve( obstaclesArray.size() );

  for(int k = 0; k < obstaclesArray.size(); ++k)
  {
    vector<point> vertices = getVerticesObstacle(obstaclesArray[k]);
    vector<point> extendedVertices = getVerticesObstacle(obstaclesArray[k]);
    vertices.pop_back();
    extendedVertices.pop_back();

    if( isPolygonClockWise(vertices) )
    {
      extendObstacle(extendedVertices, -safetyDist);
    }
    else
    {
      extendObstacle(extendedVertices, safetyDist);
    }

    bool addObstacles = false;

    for(int m = 0; m < gridCoordinates.size(); ++m)
    {
      if(pointInPolygon(extendedVertices, gridCoordinates[m]))
      {
        indx = convertPoint2Indx(gridCoordinates[m]);
        if(indx != ERR_INDX)
        {
          j = indx / nbRows;
          i = indx - (j * nbRows);
          
          if(grid[i][j] != occupied)
          {
            if(!addObstacles)
            {
              listObstacles.push_back(vertices);
              listObstacles.push_back(extendedVertices);

              addObstacles = true;
            }

            grid[i][j] = occupied;
          }        
        }
      }
    }

    if(!addObstacles)
    {
      cout << "Obstacle " << k+1 << " {" << obstaclesArray[k].x 
           << ", " << obstaclesArray[k].y << ", " << obstaclesArray[k].theta 
           << ", " << obstaclesArray[k].length << ", " << obstaclesArray[k].width 
           << "} already in the map => not added" << endl;
    }
  }

  cout << "Obstacles successfully loaded" << endl;
}


void SearchGrid::loadObstacles(std::vector< std::vector<point> > newListObstacles)
{
  cout << endl << "Loading obstacles.." << endl;

  int indx, i, j;

  for(int k = 0; k < newListObstacles.size(); ++k)
  {
    vector<point> vertices = newListObstacles[k];
    vector<point> extendedVertices = newListObstacles[k];

    if( isPolygonClockWise(vertices) )
    {
      extendObstacle(extendedVertices, -safetyDist);
    }
    else
    {
      extendObstacle(extendedVertices, safetyDist);
    }

    bool addObstacles = false;

    for(int m = 0; m < gridCoordinates.size(); ++m)
    {
      if(pointInPolygon(extendedVertices, gridCoordinates[m]))
      {
        indx = convertPoint2Indx(gridCoordinates[m]);
        if(indx != ERR_INDX)
        {
          j = indx / nbRows;
          i = indx - (j * nbRows);
          
          if(grid[i][j] != occupied)
          {
            if(!addObstacles)
            {
              listObstacles.push_back(vertices);
              listObstacles.push_back(extendedVertices);
              addObstacles = true;
            }

            grid[i][j] = occupied;
          }        
        }
      }
    }

    if(!addObstacles)
    {
      cout << "Obstacle " << k+1 << " {" ;

      for(int j = 0; j < newListObstacles[k].size(); ++j)
      {
        cout << " {" << newListObstacles[k][j].x << ", " << newListObstacles[k][j].y << "}"; 
      }
      
      cout << "} already in the map => not added" << endl;
    }
  }

  cout << "Obstacles successfully loaded" << endl;
}


bool SearchGrid::isNodeTraversable(const Node& node) const
{
  int currentIndex = convertNode2Indx(node);
  if(currentIndex == ERR_INDX)
  {
    return false;
  }

  int j = currentIndex / nbRows;
  int i = currentIndex - (j * nbRows);

  // If the node is the starting point or goal point
  if(grid[i][j] == freeC || grid[i][j] == start || grid[i][j] == goal)
  {
    return true;
  }

  return false;
}

bool SearchGrid::checkCollision(std::deque<state> line) const
{
	for (int k = 0; k < line.size(); ++k)
	{
		point pt = { line[k].x, line[k].y, line[k].z };
		int currentIndex = convertPoint2Indx(pt);
		if (currentIndex == ERR_INDX)
		{
			return false;
		}

		int j = currentIndex / nbRows;
		int i = currentIndex - (j * nbRows);

		if (grid[i][j] == occupied)
		{
			return false;
		}
	}

	return true;
}


bool SearchGrid::isGoalClose(const Node& node, double triggerDistance) const
{
  double deltaX = node.x() - goalNode->x();
  double deltaY = node.y() - goalNode->y();

  return sqrt( deltaX * deltaX + deltaY * deltaY ) < triggerDistance;
}

double SearchGrid::getDistance2Goal(const Node& node) const
{
  double deltaX = node.x() - goalNode->x();
  double deltaY = node.y() - goalNode->y();

  return sqrt( deltaX * deltaX + deltaY * deltaY );
}


bool SearchGrid::isNodeWithinGrid(const Node& node) const
{
  return (node.x() >= gridLimitsX.first && node.x() <= gridLimitsX.second && node.y() >= gridLimitsY.first  && node.y() <= gridLimitsY.second);
}


bool SearchGrid::isPointWithinGrid(const point& pt) const
{
  return (pt.x >= gridLimitsX.first && pt.x <= gridLimitsX.second && pt.y >= gridLimitsY.first  && pt.y <= gridLimitsY.second);
}


bool SearchGrid::isIndexWithinGrid(int index) const
{
  return (index >= 0 && index < nbNodes);
}


double SearchGrid::calculatePotential(const Node& node) const
{
  int nbObstacles = listObstacles.size();
  point testPoint = { node.x(), node.y() };
  vector<point> vertices;

  // To ensure that the uav is far enough from the obstacles
  double repulsivePotential = 0;
  double thresholdDistance = resolution; // 0.2 on MATLAB
  double maxPotential = 10;

  // Distance from a point to an obstacle
  double distToObstacle;

  for(int i = 0; i < nbObstacles; ++i)
  {
    vertices = listObstacles[i];

    distToObstacle = pointDistPolygon(vertices, testPoint);

    if((distToObstacle <= thresholdDistance) && (distToObstacle > 0))
    {
      repulsivePotential = repulsivePotential + 0.5 * ((1 / distToObstacle) - (1 / thresholdDistance))*((1 / distToObstacle) - (1 / thresholdDistance));
    }
    else if ((distToObstacle == thresholdDistance) || (distToObstacle <= 0))
    {
      repulsivePotential = maxPotential;
    }
  }

  if(repulsivePotential > maxPotential) {
    repulsivePotential = maxPotential;
  }

  return repulsivePotential;
}

bool SearchGrid::nodeFarFromObstacle(const Node& node) const
{
  int nbObstacles = listObstacles.size();
  point testPoint = { node.x(), node.y() };
  vector<point> vertices;

  // Distance from a point to an obstacle
  double distToObstacle;

  for(int i = 1; i < nbObstacles; i += 2)
  {
    vertices = listObstacles[i];
    distToObstacle = pointInPolygon(vertices, testPoint);

    if(distToObstacle)
    {
      return false;
    }
  }

  return true;
}


double SearchGrid::calculateHeuristic(const Node& node, heuristicType hType) const
{
  double deltaX = abs(node.x() - goalNode->x());
  double deltaY = abs(node.y() - goalNode->y());
  double heuristic;

  if(hType == MANHATTAN)
  {
    heuristic = deltaX + deltaY;
  }
  else if(hType == EUCLIDEAN)
  {
    heuristic = sqrt( deltaX * deltaX + deltaY * deltaY );
  }
  else if(hType == OCTILE)
  {
    heuristic = max(deltaX, deltaY) + (sqrt(2) - 1) * min(deltaX, deltaY);
  }

  double gridRangeX = gridLimitsX.second - gridLimitsX.first;
  double D = 1.1;
  double p = resolution / (1.4 * gridRangeX);

  return heuristic * D * (1.0 + p);
}
