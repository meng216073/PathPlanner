//  astarhybrid.cpp
//  Hybrid A* search algorithm (implementation).

#include <queue>
#include <map>
#include <iostream>
#include <chrono>

#include "astarhybrid.h"

using namespace std;
using namespace std::chrono;
using namespace astarHybrid;

AStarHybrid* AStarHybrid::create()
{
  return new AStarHybrid();
}

AStarHybrid::~AStarHybrid() {}

// Functor to compare nodes and get the one that have the lowest total cost (costSoFar + costToCome)
struct LowestCostNodes
{
  bool operator()(const Node* node1, const Node* node2) const
  {
    return node1->totalCost() > node2->totalCost();
  }
};

void AStarHybrid::setDubinsTurnRadius()
{
  // If dubins radius was not set by the user
  if(dubins.getTurnRadius() == 0)
  {
    dubins.setTurningRadius( velocity / omegaMax );
  }
}

const Node* AStarHybrid::launchSearch(const SearchGrid& searchGrid)
{
  cout << endl << endl << "A* Hybrid Algorithm starting.." << endl << endl;

  // Execution time
  auto start = high_resolution_clock::now();


  Node* startNode = const_cast<Node*>( searchGrid.getStartNode() );
  int startNodeIndx = searchGrid.convertNode2Indx(*startNode);
  if(startNodeIndx == grid::ERR_INDX)
  {
    cout <<  endl << "Incorrect start node => wrong index" << endl;
    return nullptr;
  }

  const Node* goalNode = searchGrid.getGoalNode();
  int goalNodeIndx = searchGrid.convertNode2Indx(*goalNode);
  if(goalNodeIndx == grid::ERR_INDX)
  {
    cout << endl << "Incorrect goal node => wrong index" << endl;
    return nullptr;
  }

  int nbNodes = searchGrid.getNbNodes();

  // Predecessor and successor
  Node* currentNode;
  int currentNodeIndx;
  Node* succNode;
  int succNodeIndx;

  // Total costSoFar of a given node
  double tentativeGScore = 0;

  // The set of currently discovered nodes still to be evaluated
  priority_queue<Node*, deque<Node*>, LowestCostNodes> openSet;

  // An array to keep track of the state of search agent
  map<int, Node> searchAgentMap;

  // Push on priority queue or open set
  openSet.push(startNode);
  searchAgentMap[startNodeIndx] = *startNode;

  // Dynamics
  double h = (omegaMax - (-omegaMax)) / (numberOfAnglesToDrive - 1);
  double W = -omegaMax;
  double V = velocity;
  this->setDubinsTurnRadius();

  while(!openSet.empty())
  {
	// Execution time
	high_resolution_clock::time_point current = high_resolution_clock::now();
	double elapsedTime = duration_cast<microseconds>(current - start).count() / 1.0e6;

	if (elapsedTime > 20)
		break;

    // Find the node having minimum total cost value from priority queue
    currentNode = openSet.top();
    currentNodeIndx = searchGrid.convertNode2Indx(*currentNode);

    // Node is evaluated => not going to be selected anymore
    searchAgentMap[currentNodeIndx].evaluate();
    // Remove node from open set
    openSet.pop();

    // Test if we have reached the goal
    if(currentNodeIndx == goalNodeIndx)
    {
      cout << endl << "Goal found ! (without Dubins path)" << endl;
      setPathResult(hybridAlgo::FOUND);

	  /*while (currentNode != nullptr)
	  {
		  cout << currentNode->x() << " " << currentNode->y() << " " << currentNode->theta() << " " << endl;
		  currentNode = (Node*)currentNode->pred();
	  }*/

      return currentNode;
    }
    else
    {
      // dubinsShoot
      if(isDubinsHeuristic)
      {
          // Try to see if we can reach a clean path to goal an a Dubins path
          // This is an expensive operation so do it only if we are close enough to the goal
          if(searchGrid.isGoalClose(*currentNode, dubins::TRIGGERDISTANCE))
          {
            succNode = shoot(searchGrid, *currentNode, *goalNode);

            // Only continue with dubins path if the dubins path lenght is shorter than MAXPATHLENGTH
            if(dubins.getPathLength() < dubins::MAXPATHLENGTH)
            {
              // No collision => OK
              if(succNode != nullptr)
              {
                double dist2Goal = searchGrid.getDistance2Goal(*succNode);

                if(dist2Goal <= searchGrid.getResolution())
                {
                  cout << "Goal found ! (with Dubins path)" << endl;
                  setPathResult(hybridAlgo::FOUND);
                }
                else
                {
                  cout << "Goal not found but Dubins path is close to it : " << dist2Goal << "m" << endl;
                  setPathResult(hybridAlgo::DUBINS_CLOSE);
                }

				// WRITE IN FILE HERE

				/*while(succNode != nullptr)
				{
					cout << succNode->x() << " " << succNode->y() << " " << succNode->theta() << " " << endl;
					succNode = (Node*)succNode->pred();
				}*/
				
                return succNode;
              }
            }
          }
      }

      W = -omegaMax;

      // Drive the kinematic model of the robot with different w's and get the state of the robot after each
      for(int i = 0; i < numberOfAnglesToDrive; ++i)
      {
        succNode = currentNode->findSuccessor(i, searchGrid.getResolution(), numberOfAnglesToDrive, V, W);
        W += h;

        // Check if the successor is within the grid and traversable
        if(searchGrid.isNodeWithinGrid(*succNode) && searchGrid.isNodeTraversable(*succNode))
        {
          succNodeIndx = searchGrid.convertNode2Indx(*succNode);

          // Ensure successor is not evaluated. Otherwise, it must have the same index as the predecessor
          if(!searchAgentMap[succNodeIndx].isEvaluated() || succNodeIndx == currentNodeIndx)
          {
            tentativeGScore = succNode->costSoFar() + searchGrid.calculatePotential(*succNode);

            // Ensure successor is evaluated or a shorter path was found
            if(!searchAgentMap[succNodeIndx].isEvaluated() || tentativeGScore < searchAgentMap[succNodeIndx].costSoFar() || succNodeIndx == currentNodeIndx)
            {
              findHeuristic(searchGrid, *succNode, *goalNode);

              succNode->setCostSoFar(tentativeGScore);
              succNode->setPred(currentNode);

              // If we are still on the same node and the total cost of the successor is higher
              if(succNodeIndx == currentNodeIndx && succNode->totalCost() > currentNode->totalCost())
              {
                delete succNode;
                continue;
              }
              // If we are still on the same node and the total cost of the successor is lower, set the parent to be the parent of the parent
              else if (succNodeIndx == currentNodeIndx && succNode->totalCost() <= currentNode->totalCost())
              {
                succNode->setPred(currentNode->pred());
              }

              succNode->evaluate();
              searchAgentMap[succNodeIndx] = *succNode;
              openSet.push(succNode);
			  //delete succNode;
            }
            else
            {
              delete succNode;
            }
          }
          else
          {
            delete succNode;
          }
        }
        else
        {
          delete succNode;
        }
      }
    }

  }

  setPathResult(hybridAlgo::NOT_FOUND);

  //return nullptr;
  //return goalNode;
  return startNode;
}


Node* AStarHybrid::shoot(const SearchGrid& searchGrid, const Node& startNode, const Node& goalNode)
{
  cout << "Dubins path initiated.." << endl;

  state startState = { startNode.x(), startNode.y(), startNode.theta() };
  state goalState = { goalNode.x(), goalNode.y(), goalNode.theta() };

  dubins.init(startState, goalState);

  double pathLength = dubins.getPathLength();
  deque<Node*> dubinsNodes;

  int i = 0;
  double travelledDistance = 0;

  // This should be at least half of the grid resolution
  double STEPSIZE = searchGrid.getResolution() / 2;

  while(travelledDistance < pathLength && dubinsNodes.size() < dubins::MAXNODESINPATH)
  {
      state currentState;

      if(dubins.pathSample(currentState, travelledDistance) == dubins::ERR_PARAM)
      {
        return nullptr;
      }

	  Node* newNode = new Node(currentState);
      dubinsNodes.push_back(newNode);

      if( searchGrid.isNodeTraversable(*dubinsNodes.at(i)) && searchGrid.nodeFarFromObstacle(*dubinsNodes.at(i)) )
      {
        travelledDistance = travelledDistance + STEPSIZE;

        // Set the predecessor to the previous step
        if (i > 0)
        {
          dubinsNodes.at(i)->setPred(dubinsNodes.at(i - 1));
        }
        else
        {
          dubinsNodes.at(i)->setPred(&startNode);
        }

        i++;

      }
      else
      {
        cout << "Collision detected => dubins path aborted" << endl << endl;

        for(unsigned int i = 0; i < dubinsNodes.size(); ++i)
        {
          delete dubinsNodes[i];
        }
        dubinsNodes.clear();

        return nullptr;
      }
  }

  dubinsNodes.back()->setPos(i);

  return dubinsNodes.back();
}


void AStarHybrid::findHeuristic(const SearchGrid& searchGrid, Node& startNode, const Node& goalNode)
{
  // Normal A* heuristic
  double dist2Goal = searchGrid.calculateHeuristic(startNode, heuristic);

  if(isDubinsHeuristic)
  {
    state startState = { startNode.x(), startNode.y(), startNode.theta() };
    state goalState = { goalNode.x(), goalNode.y(), goalNode.theta() };
    dubins.init(startState, goalState);
    double pathLength = dubins.getPathLength();

    startNode.setCostToCome( max(pathLength, dist2Goal) );
  }
  else
  {
    startNode.setCostToCome( dist2Goal );
  }
}
