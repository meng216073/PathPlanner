#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <thread>
#include <algorithm>

#include "inputDataConverter.h"
#include "pathplanner.h"

using namespace std;
using namespace planner;

void pathPlannerWaypoints()
{
	vector<state> startStates = {
		{ 0, 53, 0 }, { 40, 53, 0 }, { 40, 0, -M_PI/2 }, { -20, 0, M_PI }, { -20, -40, -M_PI / 2 }, { 40, -40, 0 }
	};

	vector<state> goalStates = {
		{ 40, 53, 0 }, { 40, 0, -M_PI/2 }, { -20, 0, M_PI }, { -20, -40, -M_PI / 2 }, { 40, -40, 0 }, { 0, -50, M_PI }
	};

	vector< vector<point> > obstaclesArray = {
		{ { 5, 30 },{ -5, 30 },{ -10, 35 },{ -10, 40 },{ -5, 45 },{ 5, 45 },{ 10, 40 },{ 10, 35 } },
		{ { 10, -5 },{ 20, -5 },{ 25, 0 },{ 25, 5 },{ 20, 10 },{ 10, 10 },{ 5, 5 },{ 5, 0 } },
		{ { -20, -25 },{ -10, -25 },{ -5, -20 },{ -5, -15 },{ -10, -10 },{ -20, -10 },{ -25, -15 },{ -25, -20 } }
	};

	double limitsX = 100;
	double limitsY = 110;
	double resolution = 0.6;
	double safetyDist = 4;

	for (int i = 0; i < startStates.size(); ++i)
	{
		try
		{
			PathPlanner<HybridAlgorithm> pathPlanner(startStates[i], goalStates[i], obstaclesArray, limitsX, limitsY, resolution, safetyDist);
			pathPlanner.setSearchAlgorithm("AStarHybrid");
			//pathPlanner.setSearchAlgorithmHeuristic(grid::EUCLIDEAN);

			pathPlanner.getSearchAlgorithm().setVelocity(0.5);
			pathPlanner.getSearchAlgorithm().setAngularVelocity(1.4);
			//pathPlanner.getSearchAlgorithm().setDubinsTurnRadius(0.9);
			//pathPlanner.getSearchAlgorithm().doNotUseDubins();

			if (pathPlanner.makePlan() == planner::ERR_OK)
			{
				if (pathPlanner.getPathResult() == hybridAlgo::FOUND)
				{
					cout << ":)" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::NOT_FOUND)
				{
					cout << ":(" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::DUBINS_CLOSE)
				{
					cout << ":/" << endl;
				}

				deque<state> path = pathPlanner.getPath();
			}
		}
		catch (string e)
		{
			//cout << e << endl;
		}
	}

}

void pathPlannerTEST()
{
	vector<coordinates> flyZones = {
		{ -76.428055, 38.146111, 0 },{ -76.428611, 38.151388, 0 },{ -76.431388, 38.151666, 0 },{ -76.435277, 38.150555, 0 },{ -76.432222, 38.147500, 0 },
	{ -76.432777, 38.144444, 0 },{ -76.434722, 38.143055, 0 },{ -76.432500, 38.140277, 0 },{ -76.425833, 38.140555, 0 },{ -76.421111, 38.143611, 0 },
	{ -76.423055, 38.147222, 0 },{ -76.426388, 38.146111, 0 }
	};

	vector<coordinates> waypoints = {
		{ -76.43, 38.15, 0 },{ -76.424, 38.144, 0 },{ -76.425833, 38.145555, 0 },{ -76.426388, 38.145277, 0 }
	};

	double Xmax = flyZones[0].longitude, Ymax = flyZones[0].latitude;
	double index_Xmax = 0, index_Ymax = 0;
	double Xmin = flyZones[0].longitude, Ymin = flyZones[0].latitude;
	double index_Xmin = 0, index_Ymin = 0;

	for (int i = 1; i < flyZones.size(); ++i)
	{
		if (flyZones[i].longitude > Xmax)
		{
			Xmax = flyZones[i].longitude;
			index_Xmax = i;
		}

		if (flyZones[i].latitude > Ymax)
		{
			Ymax = flyZones[i].latitude;
			index_Ymax = i;
		}

		if (flyZones[i].longitude < Xmin)
		{
			Xmin = flyZones[i].longitude;
			index_Xmin = i;
		}

		if (flyZones[i].latitude < Ymin)
		{
			Ymin = flyZones[i].latitude;
			index_Ymin = i;
		}
	}

	cout.precision(8);
	cout << flyZones[index_Xmax].longitude << " " << flyZones[index_Xmax].latitude << " Xmax " << Xmax << endl;
	cout << flyZones[index_Ymax].longitude << " " << flyZones[index_Ymax].latitude << " Ymax " << Ymax << endl;
	cout << flyZones[index_Xmin].longitude << " " << flyZones[index_Xmin].latitude << " Xmin " << Xmin << endl;
	cout << flyZones[index_Ymin].longitude << " " << flyZones[index_Ymin].latitude << " Ymin " << Ymin << endl;


	vector< vector<point> > obstaclesArray = {
		{ { 5, 30 },{ -5, 30 },{ -10, 35 },{ -10, 40 },{ -5, 45 },{ 5, 45 },{ 10, 40 },{ 10, 35 } },
	{ { 10, -5 },{ 20, -5 },{ 25, 0 },{ 25, 5 },{ 20, 10 },{ 10, 10 },{ 5, 5 },{ 5, 0 } },
	{ { -20, -25 },{ -10, -25 },{ -5, -20 },{ -5, -15 },{ -10, -10 },{ -20, -10 },{ -25, -15 },{ -25, -20 } }
	};

	double limitsX = distanceEarth(flyZones[index_Xmin].latitude, flyZones[index_Xmin].longitude, flyZones[index_Xmax].latitude, flyZones[index_Xmax].longitude);
	double limitsY = distanceEarth(flyZones[index_Ymin].latitude, flyZones[index_Ymin].longitude, flyZones[index_Ymax].latitude, flyZones[index_Ymax].longitude);

	coordinates center = { (Xmax + Xmin) / 2, (Ymax + Ymin) / 2, 0 };
	cout << "center : " << center.longitude << " " << center.latitude << endl;

	coordinates start = { waypoints[0].longitude, waypoints[0].latitude };
	coordinates goal = { waypoints[1].longitude, waypoints[1].latitude };
	//cout << start.longitude << " " << start.latitude << endl;
	cout << goal.longitude << " " << goal.latitude << endl;

	cout << "X : " << flyZones[index_Xmax].longitude << " " << center.latitude << endl;

	double startStateLon = (abs(center.longitude) - abs(waypoints[0].longitude)) * (limitsX / 2) / (abs(center.longitude) - abs(Xmax));
	double startStateLat = (abs(center.latitude) - abs(waypoints[0].latitude)) * (limitsY / 2) / (abs(center.latitude) - abs(Ymax));

	state startState = { startStateLon, startStateLat, 0 };
	cout << startState.x << endl;
	cout << startState.y << endl;

	double goalStateLon = (abs(center.longitude) - abs(waypoints[1].longitude)) * (limitsX / 2) / (abs(center.longitude) - abs(Xmax));
	double goalStateLat = (abs(center.latitude) - abs(waypoints[1].latitude)) * (limitsY / 2) / (abs(center.latitude) - abs(Ymax));

	state goalState = { goalStateLon, goalStateLat, 0 };
	cout << goalState.x << endl;
	cout << goalState.y << endl;

	double resolution = 3;
	double safetyDist = 4;

	cout << limitsX << " " << limitsY << endl;

	try
	{
		PathPlanner<HybridAlgorithm> pathPlanner(startState, goalState, obstaclesArray, limitsX, limitsY, resolution, safetyDist);
		pathPlanner.setSearchAlgorithm("AStarHybrid");
		//pathPlanner.setSearchAlgorithmHeuristic(grid::EUCLIDEAN);

		pathPlanner.getSearchAlgorithm().setVelocity(0.1);
		pathPlanner.getSearchAlgorithm().setAngularVelocity(0.1);
		//pathPlanner.getSearchAlgorithm().setDubinsTurnRadius(0.9);
		//pathPlanner.getSearchAlgorithm().doNotUseDubins();

		if (pathPlanner.makePlan() == planner::ERR_OK)
		{
			if (pathPlanner.getPathResult() == hybridAlgo::FOUND)
			{
				cout << ":)" << endl;
			}
			else if (pathPlanner.getPathResult() == hybridAlgo::NOT_FOUND)
			{
				cout << ":(" << endl;
			}
			else if (pathPlanner.getPathResult() == hybridAlgo::DUBINS_CLOSE)
			{
				cout << ":/" << endl;
			}

			deque<state> path = pathPlanner.getPath();
		}
	}
	catch (string e)
	{
		//cout << e << endl;
	}
}

void pathPlannerWaypointsNEW()
{
	vector<coordinates> flyZones = {
		{ -76.42816388888889, 38.14626944444444, 0 },{ -76.42868333333334, 38.151624999999996, 0 },{ -76.43146666666667, 38.15188888888889, 0 },{ -76.43536111111112, 38.150594444444444, 0 },{ -76.43234166666667, 38.14756666666667, 0 },
		{ -76.43294722222223, 38.144666666666666, 0 },{ -76.43476666666668, 38.143255555555555, 0 },{ -76.43263611111112, 38.14046388888889, 0 },{ -76.42601388888889, 38.14071944444444, 0 },{ -76.42120555555556, 38.14376111111111, 0 },
		{ -76.42321111111112, 38.14734722222222, 0 },{ -76.42665277777779, 38.14613055555556, 0 }
	};

	vector<coordinates> waypoints = {
		{ -76.4303750, 38.1508847, 100 }, { -76.4328839, 38.1496530, 200 }, { -76.4257010, 38.1423795, 400 }, { -76.4226473, 38.1438963, 350 }, { -76.4240670, 38.1457923, 200 }, { -76.4288839, 38.1440126, 100 }
	};

	vector<coordinatesObst> obstacles = {
		{ -76.430622, 38.149156, 100, 400 }, { -76.430576, 38.148348, 100, 400 }, { -76.430576, 38.147342, 100, 400 }, { -76.431984, 38.148638, 100, 400 }, { -76.428997, 38.140578, 300, 750 }, { -76.431974, 38.150264, 100, 600 },
		{ -76.430952, 38.150481, 100, 400 }, { -76.430606, 38.148071, 100, 400 }, { -76.430789, 38.146779, 200, 400 }, { -76.429984, 38.147159, 100, 400 }, { -76.429351, 38.145961, 100, 400 }, { -76.429662, 38.144856, 100, 400 },
		{ -76.427763, 38.144746, 200, 400 }, { -76.425660, 38.145615, 300, 400 }, { -76.423042, 38.144664, 200, 400 }, { -76.426522, 38.143163, 200, 400 }, { -76.423797, 38.142969, 200, 400 }
	};

	double Xmax = flyZones[0].longitude, Ymax = flyZones[0].latitude;
	double index_Xmax = 0, index_Ymax = 0;
	double Xmin = flyZones[0].longitude, Ymin = flyZones[0].latitude;
	double index_Xmin = 0, index_Ymin = 0;

	for (int i = 1; i < flyZones.size(); ++i)
	{
		if (flyZones[i].longitude > Xmax)
		{
			Xmax = flyZones[i].longitude;
			index_Xmax = i;
		}

		if (flyZones[i].latitude > Ymax)
		{
			Ymax = flyZones[i].latitude;
			index_Ymax = i;
		}

		if (flyZones[i].longitude < Xmin)
		{
			Xmin = flyZones[i].longitude;
			index_Xmin = i;
		}

		if (flyZones[i].latitude < Ymin)
		{
			Ymin = flyZones[i].latitude;
			index_Ymin = i;
		}
	}

	cout.precision(8);
	cout << flyZones[index_Xmax].longitude << " " << flyZones[index_Xmax].latitude << " Xmax " << Xmax << endl;
	cout << flyZones[index_Ymax].longitude << " " << flyZones[index_Ymax].latitude << " Ymax " << Ymax << endl;
	cout << flyZones[index_Xmin].longitude << " " << flyZones[index_Xmin].latitude << " Xmin " << Xmin << endl;
	cout << flyZones[index_Ymin].longitude << " " << flyZones[index_Ymin].latitude << " Ymin " << Ymin << endl;

	double limitsX = distanceEarth(flyZones[index_Xmin].latitude, flyZones[index_Xmin].longitude, flyZones[index_Xmax].latitude, flyZones[index_Xmax].longitude);
	double limitsY = distanceEarth(flyZones[index_Ymin].latitude, flyZones[index_Ymin].longitude, flyZones[index_Ymax].latitude, flyZones[index_Ymax].longitude);

	coordinates center = { (Xmax + Xmin) / 2, (Ymax + Ymin) / 2, 0 };
	cout << "center : " << center.longitude << " " << center.latitude << endl;

	vector<state> startStates, goalStates;
	state stateWaypoint = toCartesianState(waypoints[0], center, limitsX, limitsY, Xmax, Ymax);
	startStates.push_back(stateWaypoint);

	int i = 1;
	for (i; i < waypoints.size() - 1; ++i)
	{
		stateWaypoint = toCartesianState(waypoints[i], center, limitsX, limitsY, Xmax, Ymax);

		goalStates.push_back(stateWaypoint);
		startStates.push_back(stateWaypoint);
	}

	stateWaypoint = toCartesianState(waypoints[i], center, limitsX, limitsY, Xmax, Ymax);
	goalStates.push_back(stateWaypoint);

	for (int i = 0; i < startStates.size(); ++i)
	{
		cout << startStates[i].x << " " << startStates[i].y << " **** " << goalStates[i].x << " " << goalStates[i].y << endl;
	}
	

	// SAVE OBSTACLES IN FILE
	ofstream obstacleCartesianFile(OBSTACLES_CARTESIAN_FILE, std::ios_base::app);

	for (int i = 0; i < obstacles.size(); ++i)
	{
		coordinates cylinder = coordinates{ obstacles[i].longitude, obstacles[i].latitude, obstacles[i].height };
		point point = toCartesianPoint(cylinder, center, limitsX, limitsY, Xmax, Ymax);
		double radius = feet2met(obstacles[i].radius);

		obstacleCartesianFile << point.x << "," << point.y << "," << point.z << "," << radius << endl;
	}

	// OBSTACLE TO POLYGON TEST
	ofstream obstacleGpsFile(OBSTACLES_GPS_FILE, std::ios_base::app);

	vector< vector<point> > obstaclesArray;
	for (int i = 0; i < obstacles.size(); ++i)
	{
		vector<coordinates> obstacle2Polygon = GenerateCirclePolygon(obstacles[i]);

		for (int i = 0; i < obstacle2Polygon.size(); ++i)
		{
			obstacleGpsFile << obstacle2Polygon[i].latitude << "," << obstacle2Polygon[i].longitude << endl;
		}

		vector<point> newObstaclePolygon = vecPoint2vecCoord(obstacle2Polygon, center, limitsX, limitsY, Xmax, Ymax);
		obstaclesArray.push_back(newObstaclePolygon);
	}
	

	/*
	// Multithreading
	vector<pair<state, state>> startGoalVector;

	for (int i = 0; i < startStates.size(); ++i)
	{
		startGoalVector.push_back(make_pair(startStates[i], goalStates[i]));
	}

	vector<thread> threads;

	for_each(startGoalVector.begin(), startGoalVector.end(), [&threads, obstaclesArray, limitsX, limitsY, resolution, safetyDist](auto statePair) {
		threads.emplace_back(thread{
			[statePair, obstaclesArray, limitsX, limitsY, resolution, safetyDist]() {

			PathPlanner<HybridAlgorithm> pathPlanner((state&)statePair.first, (state&)statePair.second, (vector< vector<point> >&)obstaclesArray, limitsX, limitsY, resolution, safetyDist);
			pathPlanner.loadObstacles(obstaclesArray);

			pathPlanner.setSearchAlgorithm("AStarHybrid");
			//pathPlanner.setSearchAlgorithmHeuristic(grid::EUCLIDEAN);

			pathPlanner.getSearchAlgorithm().setVelocity(0.1);
			pathPlanner.getSearchAlgorithm().setAngularVelocity(0.1);
			//pathPlanner.getSearchAlgorithm().setDubinsTurnRadius(0.9);
			//pathPlanner.getSearchAlgorithm().doNotUseDubins();

			if (pathPlanner.makePlan() == planner::ERR_OK)
			{
				if (pathPlanner.getPathResult() == hybridAlgo::FOUND)
				{
					cout << ":)" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::NOT_FOUND)
				{
					cout << ":(" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::DUBINS_CLOSE)
				{
					cout << ":/" << endl;
				}

				deque<state> path = pathPlanner.getPath();
			}

		}
			});
	});

	for (int i = 0; i < threads.size(); i++)
		threads[i].join();
	*/
	



	//makeNextPlan(const state& startState, const state& goalState)

	/*PathPlanner<HybridAlgorithm> pathPlanner(startStates[0], goalStates[0], obstaclesArray, limitsX, limitsY, resolution, safetyDist);
	pathPlanner.loadObstacles(obstaclesArray);

	pathPlanner.setSearchAlgorithm("AStarHybrid");
	//pathPlanner.setSearchAlgorithmHeuristic(grid::MANHATTAN);

	pathPlanner.getSearchAlgorithm().setVelocity(100);
	pathPlanner.getSearchAlgorithm().setAngularVelocity(25);
	//pathPlanner.getSearchAlgorithm().setDubinsTurnRadius(0.9);
	//pathPlanner.getSearchAlgorithm().doNotUseDubins();

	// Next path planners
	for (int i = 0; i < startStates.size(); ++i)
	{
		try
		{
			pathPlanner.setStartState(startStates[i]);
			pathPlanner.setGoalState(goalStates[i]);

			if (pathPlanner.makePlan() == planner::ERR_OK)
			{
				if (pathPlanner.getPathResult() == hybridAlgo::FOUND)
				{
					cout << ":)" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::NOT_FOUND)
				{
					cout << ":(" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::DUBINS_CLOSE)
				{
					cout << ":/" << endl;
				}

				deque<state> path = pathPlanner.getPath();
			}
		}
		catch (string e)
		{
			//cout << e << endl;
		}
	}*/

	double resolution = 1; // 4
	double safetyDist = 5;

	// Bearing
	for (int i = 0; i < startStates.size(); ++i)
		computeAngle2Points(startStates[i], goalStates[i]);

	// Next path planners
	for (int i = 0; i < startStates.size(); ++i)
	{
		try
		{
			PathPlanner<HybridAlgorithm> pathPlanner(startStates[i], goalStates[i], obstaclesArray, limitsX, limitsY, resolution, safetyDist);

			pathPlanner.setSearchAlgorithm("AStarHybrid");
			//pathPlanner.setSearchAlgorithmHeuristic(grid::MANHATTAN);

			pathPlanner.getSearchAlgorithm().setVelocity(1); // 100
			pathPlanner.getSearchAlgorithm().setAngularVelocity(0.25); // 25
			pathPlanner.getSearchAlgorithm().setDubinsTurnRadius(0.1); // 1
			//pathPlanner.getSearchAlgorithm().doNotUseDubins();

			if (pathPlanner.makePlan() == planner::ERR_OK)
			{
				if (pathPlanner.getPathResult() == hybridAlgo::FOUND)
				{
					cout << ":)" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::NOT_FOUND)
				{
					cout << ":(" << endl;
				}
				else if (pathPlanner.getPathResult() == hybridAlgo::DUBINS_CLOSE)
				{
					cout << ":/" << endl;
				}

				deque<state> path = pathPlanner.getPath();

				// Waypoints gps file
				ofstream waypointsGpsFile(WAYPOINTS_GPS_FILE, std::ios_base::app);

				for (int i = 0; i < path.size(); ++i)
				{
					coordinates coord = toCoordinates(path[i], center, limitsX, limitsY, Xmax, Ymax);
					waypointsGpsFile << coord.latitude << "," << coord.longitude << "," << coord.altitude * meter2feet << endl;
				}
				
				if(i == startStates.size() - 1)
				{
					coordinates coord = toCoordinates(goalStates[i], center, limitsX, limitsY, Xmax, Ymax);
					waypointsGpsFile << coord.latitude << "," << coord.longitude << "," << coord.altitude * meter2feet << endl;
				}
				
			}
		}
		catch (string e)
		{
			//cout << e << endl;
		}
	}
}


int main()
{
  //pathPlannerTEST();
  //pathPlannerWaypoints();
  pathPlannerWaypointsNEW();

  return 0;
}
