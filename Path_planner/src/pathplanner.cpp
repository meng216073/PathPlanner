//  pathplanner.h
//  Main class (implementation).

#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "pathplanner.h"
#include "algorithmfactory.h"
#include "util.h"

using namespace std;
using namespace std::chrono;
using namespace planner;

high_resolution_clock::time_point t1, t2;


// Algorithm families
template class PathPlanner<HybridAlgorithm>;
template class PathPlanner<BasicAlgorithm>;

template <class T>
PathPlanner<T>::PathPlanner(state& IstartState, state& IgoalState, vector<obstacle>& IobstaclesArray,
                            double gridLengthX, double gridLengthY, double resolution, double safetyDist)
{
  
  // Execution time
  t1 = high_resolution_clock::now();

  pair<double,double> gridLimitsX(-gridLengthX / 2, gridLengthX / 2);
  pair<double,double> gridLimitsY(-gridLengthY / 2, gridLengthY / 2);

  cout << endl << "*** Input data validation.. ***" << endl;

  if(checkInputs(IstartState, IgoalState, gridLimitsX, gridLimitsY, resolution, safetyDist) != ERR_OK)
  {
    throw string("|!| ABORT : WRONG INPUTS |!|");
  }
  else
  {
    int errorObst = checkObstacles(IobstaclesArray, gridLimitsX, gridLimitsY);

    if(errorObst == ERR_OK)
    {
      cout << "*** Correct input data validation ***" << endl;
    }
  }

  smoother = unique_ptr<Smoother>( new Smoother() );
  searchGrid = unique_ptr<SearchGrid>( new SearchGrid(IstartState, IgoalState, gridLimitsX, gridLimitsY, resolution) );

  searchGrid->setSafetyDist(safetyDist);

  if(searchGrid->createWorkspace() == ERR_OK)
  {
    searchGrid->loadObstacles(IobstaclesArray);
    cout << endl << "Map created" << endl;
  }
  else
  {
    cout << "Impossible to create the workspace" << endl;
  }

  searchAlgorithm = nullptr;
}

template <class T>
int PathPlanner<T>::checkInputs(state& IstartState, state& IgoalState, std::pair<double,double> gridLimitsX, std::pair<double,double> gridLimitsY, double resolution, double safetyDist) const
{
  if(IstartState.x == IgoalState.x && IstartState.y == IgoalState.y)
  {
    cout << "Start position = Goal position" << endl;
    return ERR_POS_EQUAL;
  }

  if(gridLimitsX.first >= 0)
  {
    cout << "Interval X <= 0" << endl;
    return ERR_LIMITS_X;
  }

  if(gridLimitsY.first >= 0)
  {
    cout << "Interval Y <= 0" << endl;
    return ERR_LIMITS_Y;
  }

  if(IstartState.x < gridLimitsX.first || IstartState.x > gridLimitsX.second || IstartState.y < gridLimitsY.first || IstartState.y > gridLimitsY.second)
  {
    cout << "Start position outside the map" << endl;
    return ERR_START_POS;
  }

  if(IgoalState.x < gridLimitsY.first || IgoalState.x > gridLimitsY.second || IgoalState.y < gridLimitsY.first || IgoalState.y > gridLimitsY.second)
  {
    cout << "Goal position outside the map" << endl;
    return ERR_GOAL_POS;
  }

  if(resolution <= 0)
  {
    cout << "Resolution <= 0" << endl;
    return ERR_RESOLUTION;
  }

  if(IstartState.theta < 0 || IstartState.theta > 2*M_PI)
  {
    IstartState.theta = foldOverTwoPi(IstartState.theta);
  }

  if(IgoalState.theta < 0 || IgoalState.theta > 2*M_PI)
  {
    IgoalState.theta = foldOverTwoPi(IgoalState.theta);
  }

  return ERR_OK;
}

template <class T>
int PathPlanner<T>::checkObstacles(std::vector<obstacle>& IobstaclesArray, std::pair<double,double> gridLimitsX, std::pair<double,double> gridLimitsY) const
{
  cout << "Checking obstacles.." << endl;

  double x, y, theta, length, width;

  for(int i = 0; i < IobstaclesArray.size(); ++i)
  {
    x = IobstaclesArray[i].x;
    y = IobstaclesArray[i].y;
    theta = abs(IobstaclesArray[i].theta);
    length = IobstaclesArray[i].length;
    width = IobstaclesArray[i].width;

    if(length < 0)
    {
      cout << "Obstacle " << i+1 << " {" << x << ", " << y << ", " << theta << ", " << length << ", " << width << "} : length < 0" << endl;
      return ERR_LENGTH;
    }

    if(width < 0)
    {
      cout << "Obstacle " << i+1 << " {" << x << ", " << y << ", " << theta << ", " << length << ", " << width << "} : width < 0" << endl;
      return ERR_WIDTH;
    }

    if( (x + (length / 2) * cos(theta)) + sin(theta) * (width / 2) <= gridLimitsX.first)
    {
        cout << "Obstacle " << i+1 << " {" << x << ", " << y << ", " << theta << ", " << length << ", " << width << "} outside the map (x value) => deleted" << endl;
        IobstaclesArray.erase(IobstaclesArray.begin() + i);
        i--;
    }
    else if( (x - (length / 2) * cos(theta)) - sin(theta) * (width / 2) >= gridLimitsX.second)
    {
        cout << "Obstacle " << i+1 << " {" << x << ", " << y << ", " << theta << ", " << length << ", " << width << "} outside the map (x value) => deleted" << endl;
        IobstaclesArray.erase(IobstaclesArray.begin() + i);
        i--;
    }
    else if( (y + (length / 2) * sin(theta)) + cos(theta) * (width / 2) <= gridLimitsY.first)
    {
      cout << "Obstacle " << i+1 << " {" << x << ", " << y << ", " << theta << ", " << length << ", " << width << "} outside the map (y value) => deleted" << endl;
      IobstaclesArray.erase(IobstaclesArray.begin() + i);
      i--;
    }
    else if( (y - (length / 2) * sin(theta)) - cos(theta) * (width / 2) >= gridLimitsY.second)
    {
      cout << "Obstacle " << i+1 << " {" << x << ", " << y << ", " << theta << ", " << length << ", " << width << "} outside the map (y value) => deleted" << endl;
      IobstaclesArray.erase(IobstaclesArray.begin() + i);
      i--;
    }
  }

  cout << "Obstacles successfully checked" << endl;

  return ERR_OK;
}


template <class T>
PathPlanner<T>::PathPlanner(state& IstartState, state& IgoalState, std::vector< std::vector<point> >& IobstaclesArray,
                            double gridLengthX, double gridLengthY, double resolution, double safetyDist)
{
  // Execution time
  t1 = high_resolution_clock::now();

  pair<double,double> gridLimitsX(-gridLengthX / 2, gridLengthX / 2);
  pair<double,double> gridLimitsY(-gridLengthY / 2, gridLengthY / 2);

  /*cout << endl << "*** Input data validation.. ***" << endl;

  if(checkInputs(IstartState, IgoalState, gridLimitsX, gridLimitsY, resolution, safetyDist) != ERR_OK)
  {
    throw string("|!| ABORT : WRONG INPUTS |!|");
  }
  else
  {
    checkObstacles(IobstaclesArray, gridLimitsX, gridLimitsY);
    cout << "*** Correct input data validation ***" << endl;
  }*/

  smoother = unique_ptr<Smoother>( new Smoother() );
  searchGrid = unique_ptr<SearchGrid>( new SearchGrid(IstartState, IgoalState, gridLimitsX, gridLimitsY, resolution) );

  searchGrid->setSafetyDist(safetyDist);

  if(searchGrid->createWorkspace() == ERR_OK)
  {
    searchGrid->loadObstacles(IobstaclesArray);
    cout << endl << "Map created" << endl;
  }
  else
  {
    cout << "Impossible to create the workspace" << endl;
  }

  searchAlgorithm = nullptr;
}

template <class T>
void PathPlanner<T>::checkObstacles(std::vector< std::vector<point> >& IobstaclesArray, std::pair<double,double> gridLimitsX, std::pair<double,double> gridLimitsY) const
{
  cout << "Checking obstacles.." << endl;

  double x, y;
  unsigned int obstOutside = 0;

  for(int i = 0; i < IobstaclesArray.size(); ++i)
  {
    for(int j = 0; j < IobstaclesArray[i].size(); ++j)
    {
      x = IobstaclesArray[i][j].x;
      y = IobstaclesArray[i][j].y;

      if(x < gridLimitsX.first || x > gridLimitsX.second || y < gridLimitsY.first || y > gridLimitsY.second)
      {
        obstOutside++;
      }
    }

    if(obstOutside == IobstaclesArray[i].size())
    {
      cout << "Obstacle " << i+1 << " {" ;
      for(int j = 0; j < IobstaclesArray[i].size(); ++j)
      {
        cout << " {" << IobstaclesArray[i][j].x << ", " << IobstaclesArray[i][j].y << "}"; 
      }
      cout << "} outside the map => deleted" << endl;

      IobstaclesArray.erase(IobstaclesArray.begin() + i);
    }
  }

  cout << "Obstacles successfully checked" << endl;
}

template <>
void PathPlanner<HybridAlgorithm>::setSearchAlgorithm(const string& name)
{
  cout << "Search algorithm family : Hybrid" << endl;
  searchAlgorithm = HybridAlgorithmFactory::getFactory().getSearchAlgorithm(name);
}

template <>
void PathPlanner<BasicAlgorithm>::setSearchAlgorithm(const string& name)
{
  cout << "Search algorithm family : Basic" << endl;
  searchAlgorithm = BasicAlgorithmFactory::getFactory().getSearchAlgorithm(name);
}

template <class T>
int PathPlanner<T>::setSearchAlgorithmHeuristic(grid::heuristicType hType)
{
  if(searchAlgorithm == nullptr)
  {
    cout << "SearchAlgorithm not initialized (setSearchAlgorithmHeuristic)" << endl;
    return ERR_ALGO;
  }

  searchAlgorithm->setHeuristic(hType);
}

template <class T>
int PathPlanner<T>::setListObstacles(std::vector<obstacle>& newListObstacles)
{
  if(searchGrid == nullptr)
  {
    cout << "SearchGrid not created (setListObstacles)" << endl;
    return ERR_GRID;
  }

  cout << endl << "*** Adding obstacles.. ***" << endl;

  checkObstacles(newListObstacles, searchGrid->getGridLimitsX(), searchGrid->getGridLimitsY());
  searchGrid->loadObstacles(newListObstacles);

  cout << "*** Obstacles successfully added ***" << endl;
}

template <class T>
int PathPlanner<T>::setListObstacles(std::vector< std::vector<point> >& newListObstacles)
{
  if(searchGrid == nullptr)
  {
    cout << "SearchGrid not created (setListObstacles)" << endl;
    return ERR_GRID;
  }

  cout << endl << "*** Adding obstacles.. ***" << endl;

  checkObstacles(newListObstacles, searchGrid->getGridLimitsX(), searchGrid->getGridLimitsY());
  searchGrid->loadObstacles(newListObstacles);

  cout << "*** Obstacles successfully added ***" << endl;
}

template <class T>
int PathPlanner<T>::setDist2Obstacles(double dist)
{
  if(searchGrid == nullptr)
  {
    cout << "SearchGrid not created (setDist2Obstacles)" << endl;
    return ERR_GRID;
  }

  searchGrid->setSafetyDist(dist);
}

template <class T>
int PathPlanner<T>::makePlan()
{
  if(searchAlgorithm == nullptr)
  {
    cout << "SearchAlgorithm not initialized (makePlan)" << endl;
    return ERR_ALGO;
  }

  const Node* nodePath = searchAlgorithm->launchSearch(*searchGrid);

  /*cout << nodePath->pos() << endl;
  int i = 0;
  while (nodePath != nullptr)
  {
	  cout << nodePath->x() << " " << i << endl;
	  ++i;
	  nodePath = nodePath->pred();
  }*/

  if(!nodePath)
  {
    cout << endl << endl << "*** PATH NOT FOUND ***" << endl;
    return ERR_OK; // Not a program error (normal behavior)
  }
  else
  {
    cout << endl << endl << "*** PATH FOUND ***" << endl;
  }

  // Smoother
  finalPath = smoother->getPathFromGoalNode(nodePath);
  finalPath = smoother->smooth(finalPath);

  // Reduce nb waypoints
  reduceNbWaypoints(finalPath);

  // Set altitude
  setWaypointsAltitude(finalPath);

  // Delete path
  deletePath(nodePath);

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto execTime = duration_cast<microseconds>( t2 - t1 ).count() / 1.0e6;

  cout << endl << "Execution time : " << execTime << "s" << endl;

  // Save results
  saveOutputs(execTime);

  return ERR_OK;
}

template <class T>
int PathPlanner<T>::makeNextPlan(const state& startState, const state& goalState)
{
	if (searchAlgorithm == nullptr)
	{
		cout << "SearchAlgorithm not initialized (makePlan)" << endl;
		return ERR_ALGO;
	}

	searchGrid->setStartNode(startState);
	searchGrid->setGoalNode(goalState);

	const Node* nodePath = searchAlgorithm->launchSearch(*searchGrid);

	if (!nodePath)
	{
		cout << endl << endl << "*** PATH NOT FOUND ***" << endl;
		return ERR_OK; // Not a program error (normal behavior)
	}
	else
	{
		cout << endl << endl << "*** PATH FOUND ***" << endl;
	}

	// Smoother
	finalPath = smoother->getPathFromGoalNode(nodePath);
	finalPath = smoother->smooth(finalPath);

	cout << finalPath.size() << endl;

	// Delete path
	deletePath(nodePath);

	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	auto execTime = duration_cast<microseconds>(t2 - t1).count() / 1.0e6;

	cout << endl << "Execution time : " << execTime << "s" << endl;

	// Save results
	saveOutputs(execTime);

	return ERR_OK;
}

template <class T>
void PathPlanner<T>::setWaypointsAltitude(std::deque<state>& path) const
{
	double startAlt = searchGrid->getStartState().z;
	double goalAlt = searchGrid->getGoalState().z;

	double diff = abs(goalAlt - startAlt);
	double deltaAlt = diff / path.size();

	path[0].z = startAlt;

	for (int i = 1; i < path.size(); ++i)
	{
		if (startAlt < goalAlt)
			path[i].z = path[i - 1].z + deltaAlt;
		else
			path[i].z = path[i - 1].z - deltaAlt;
	}
}

template <class T>
void PathPlanner<T>::reduceNbWaypoints(std::deque<state>& path) const
{
	std::deque<state> line, newPath;
	bool noCollision = true;
	int nb = 0;
	state nextState, firstState = path[nb];
	newPath.push_back(firstState);

	while (nb < path.size() - 1)
	{
		// If there is not a collision
		if (noCollision)
		{
			nb++;
			nextState = path[nb];

			double a = (nextState.y - firstState.y) / (nextState.x - firstState.x);
			double b = firstState.y - a * firstState.x;

			double distanceX = abs(nextState.x - firstState.x);
			double minCoord = std::min(firstState.x, nextState.x);

			for (int i = 0; i <= distanceX; i++)
			{
				line.push_back({ minCoord + i , a * (minCoord + i) + b, nextState.theta, 0 });
			}

			noCollision = searchGrid->checkCollision(line);

			line.clear();
		}
		else // Collision
		{
			newPath.push_back(path[nb - 1]);
			firstState = path[nb];

			noCollision = true;
		}
	}

	path = newPath;
}

template <class T>
void PathPlanner<T>::deletePath(const Node* goalNode) const
{
  int i = 0;
  int pos = goalNode->pos();

  while(i < pos)
  {
    const Node* previousNode = goalNode->pred();
    delete goalNode;
    goalNode = previousNode;
    i++;
  }

}

template <class T>
void PathPlanner<T>::saveOutputs(double execTime)
{
  ofstream waypointsFile(WAYPOINTS_FILE, std::ios_base::app);
  //ofstream waypointsFile(WAYPOINTS_FILE);
  ofstream obstaclesFile(OBSTACLES_FILE);
  ofstream mapFile(MAP_FILE);

  if(!waypointsFile || !obstaclesFile || !mapFile)
  {
    cout << endl << "Folder <" << OUTPUTS_DIR << "> does not exist => path planner outputs not saved" << endl;
    return;
  }

  mapFile << searchGrid->getSafetyDist() << endl;
  mapFile << searchGrid->getResolution() << endl;
  mapFile << searchGrid->getGridLimitsX().first << "," << searchGrid->getGridLimitsX().second << "," << searchGrid->getGridLimitsY().first << "," << searchGrid->getGridLimitsY().second << endl;
  
  waypointsFile << finalPath.size() << endl;
  waypointsFile << execTime << endl;

  waypointsFile << searchGrid->getStartState().x << "," << searchGrid->getStartState().y << "," << searchGrid->getStartState().theta << "," << searchGrid->getStartState().z << endl;

  for(deque<state>::iterator it = finalPath.begin() + 1; it != finalPath.end(); ++it)
  {
    waypointsFile << it->x << "," << it->y << "," << it->theta << "," << it->z << endl;
  }

  waypointsFile << searchGrid->getGoalState().x << "," << searchGrid->getGoalState().y << "," << searchGrid->getGoalState().theta << "," << searchGrid->getGoalState().z << endl;

  vector< vector<point> > obstaclesVertices = this->getObstaclesVertices();
  for(int i = 0; i < obstaclesVertices.size(); ++i)
  {
    for(int j = 0; j < obstaclesVertices[i].size(); ++j)
    {
      obstaclesFile << obstaclesVertices[i][j].x << "," << obstaclesVertices[i][j].y;

      if(j != obstaclesVertices[i].size() - 1)
      {
        obstaclesFile << ",";
      }
    }

    obstaclesFile << endl;
  }

  cout << endl << endl << "Results saved in folder : <" << OUTPUTS_DIR << ">" << endl;
}