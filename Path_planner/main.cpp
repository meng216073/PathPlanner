#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
//#include <thread>
#include <algorithm>
#include <iomanip>   
#include <random>
#include <tuple>

#include "inputDataConverter.h"
#include "pathplanner.h"

using namespace std;
using namespace planner;

tuple<vector<coordinates>, vector<coordinatesObst>> getRandomConfig(double latMin, double lonMin, double latMax, double lonMax, double altMin, double altMax, double radMin, double radMax)
{
	cout.precision(15);

	// 6-20 waypoints
	unsigned int nbWaypoints = 15;
	vector<coordinates> waypoints;
	waypoints.reserve(nbWaypoints);

	random_device rd;  //Will be used to obtain a seed for the random number engine
	mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	uniform_real_distribution<> lonDis(lonMin, lonMax);
	uniform_real_distribution<> latDis(latMin, latMax);
	uniform_real_distribution<> altDis(altMin, altMax);

	// 15-30 obstacles
	unsigned int nbObstacles = 30;
	vector<coordinatesObst> obstacles;
	obstacles.reserve(nbObstacles);

	uniform_real_distribution<> radDis(radMin, radMax);

	for (int i = 0; i < nbObstacles; ++i)
	{
		// Use dis to transform the random unsigned int generated by gen into a 
		// double in [1, 2). Each call to dis(gen) generates a new random double

		obstacles.push_back({ lonDis(gen), latDis(gen), radDis(gen), altDis(gen) });
		//cout << obstacles[i].longitude << " " << obstacles[i].latitude << " " << obstacles[i].radius << " " << obstacles[i].height << endl;
	}

	for (int i = 0; i < nbWaypoints; ++i)
	{
		// Use dis to transform the random unsigned int generated by gen into a 
		// double in [1, 2). Each call to dis(gen) generates a new random double
		int add = 1;
		double lon = lonDis(gen);
		double lat = latDis(gen);
		double alt = altDis(gen);
		double d = 0;

		// Check if a waypoint is inside an obstacle
		for (int j = 0; j < nbObstacles; ++j)
		{
			d = distanceEarth(lat, lon, obstacles[j].latitude, obstacles[j].longitude);

			if (d*feet2meter <= obstacles[j].radius && alt <= obstacles[j].height)
			{
				add = 0;
			}

			// Check the distance between two waypoints
			for (int k = 0; k < nbWaypoints; ++k)
			{
				d = distanceEarth(lat, lon, waypoints[k].latitude, waypoints[k].longitude);

				if (d <= 50)
				{
					add = 0;
				}
			}
		}

		if(add == 1)
			waypoints.push_back({ lon, lat, alt });

		cout << waypoints[i].longitude << " " << waypoints[i].latitude << " " << waypoints[i].altitude << endl;
	}

	return make_tuple(waypoints, obstacles);
}

void pathPlannerWaypoints()
{
	// CONFIG COMPE
	/*vector<coordinates> flyZones = {
		{ -76.42816388888889, 38.14626944444444, 0 },{ -76.42868333333334, 38.151624999999996, 0 },{ -76.43146666666667, 38.15188888888889, 0 },{ -76.43536111111112, 38.150594444444444, 0 },{ -76.43234166666667, 38.14756666666667, 0 },
		{ -76.43294722222223, 38.144666666666666, 0 },{ -76.43476666666668, 38.143255555555555, 0 },{ -76.43263611111112, 38.14046388888889, 0 },{ -76.42601388888889, 38.14071944444444, 0 },{ -76.42120555555556, 38.14376111111111, 0 },
		{ -76.42321111111112, 38.14734722222222, 0 },{ -76.42665277777779, 38.14613055555556, 0 }
	};*/

	/*vector<coordinates> waypoints = {
		{ -76.4303750, 38.1508847, 100 },{ -76.4328839, 38.1496530, 200 },{ -76.4257010, 38.1423795, 400 },{ -76.4226473, 38.1438963, 350 },{ -76.4240670, 38.1457923, 200 },{ -76.4288839, 38.1440126, 100 }
	};*/
	
	 
	/*vector<coordinates> waypoints = {
		{ -76.429103, 38.150842, 600 }, { -76.424003, 38.146664, 600 }, { -76.428920, 38.151226, 600 }, { -76.423042, 38.144664, 600 }, { -76.427763, 38.144746, 450 }, { -76.430622, 38.149156, 450 }, { -76.4303750, 38.1508847, 100 }, { -76.4328839, 38.1496530, 200 },{ -76.4257010, 38.1423795, 400 },{ -76.4226473, 38.1438963, 350 },{ -76.4240670, 38.1457923, 200 },{ -76.4288839, 38.1440126, 100 }
	};
	
	vector<coordinatesObst> obstacles = {
		{ -76.430622, 38.149156, 100, 400 }, { -76.430576, 38.148348, 100, 400 }, { -76.430576, 38.147342, 100, 400 }, { -76.431984, 38.148638, 100, 400 }, { -76.428997, 38.140578, 300, 750 },{ -76.431974, 38.150264, 100, 600 },
		{ -76.430952, 38.150481, 100, 400 }, { -76.430606, 38.148071, 100, 400 }, { -76.430789, 38.146779, 200, 400 }, { -76.429984, 38.147159, 100, 400 }, { -76.429351, 38.145961, 100, 400 },{ -76.429662, 38.144856, 100, 400 },
		{ -76.427763, 38.144746, 200, 400 }, { -76.425660, 38.145615, 300, 400 }, { -76.423042, 38.144664, 200, 400 }, { -76.426522, 38.143163, 200, 400 }, { -76.423797, 38.142969, 200, 400 }
	};*/



	// CONFIG CHALET ALL FLY ZONE
	vector<coordinates> flyZones = {
		{ -74.432358, 45.405951 , 0 }, { -74.433669, 45.406155, 0 }, { -74.432425, 45.409364, 0 }, { -74.431047, 45.409148, 0 }
	};

	vector<coordinates> waypoints = {
		{ -74.432698, 45.406083, 100 },
		{ -74.432501, 45.408832, 150 },
		{ -74.432738, 45.408272, 250 }

		/*{ -74.43223294607526, 45.407381364890156, 200 },
		{ -74.43158477210385, 45.40815327259285, 250 },
		{ -74.431690, 45.407910, 250 },
		{ -74.43254502982818, 45.40835214666499, 300 },
		{ -74.43206490097482, 45.409124041101464, 350 },
		{ -74.43140232314875, 45.40896898891854, 400 },
		{ -74.43166639400576, 45.40847349321838, 350 },
		{ -74.432115, 45.408398, 450 },
		{ -74.43271787620766, 45.408089228254845, 300 },
		{ -74.43209805262615, 45.40700695034278, 200 },
		{ -74.432102, 45.408286, 450 },
		{ -74.43252859911847, 45.406080760915515, 100 }*/
	};

	/*vector<coordinates> waypoints = {
		{ -74.43269867107395, 45.40608359235939, 100 },
		{ -74.43280910072043, 45.40753305064806, 150 },
		{ -74.43223294607526, 45.407381364890156, 200 },
		{ -74.43158477210385, 45.40815327259285, 250 },
		{ -74.43254502982818, 45.40835214666499, 300 },
		{ -74.43206490097482, 45.409124041101464, 300 },
		{ -74.43140232314875, 45.40896898891854, 300 },
		{ -74.43140232314875, 45.40896898891854, 300 },
		{ -74.43166639400576, 45.40847349321838, 250 },
		{ -74.43271787620766, 45.408089228254845, 200 },
		{ -74.43209805262615, 45.40700695034278, 150 },
		{ -74.43252859911847, 45.406080760915515, 100 }
	};*/

	/*vector<coordinates> waypoints = {
		{ -74.43168, 45.40794, 100 },
		{ -74.43169, 45.40791, 150 },
		{ -74.43188, 45.40800, 200 },
		{ -74.43188, 45.40800, 250 },
		{ -74.43188, 45.40800, 300 },
		{ -74.43188, 45.40800, 350 },
		{ -74.43460, 45.40792, 400 },
	};*/

	
	/*vector<coordinatesObst> obstacles = {
		{ -74.43264370438719, 45.40784940730905, 65.9, 400 },
		{ -74.43205399457125, 45.407688578685686, 62.74084156720309, 400 },
		{ -74.43271756283168, 45.40710974534067, 117.04832333456076, 400 },
		{ -74.43210200701087, 45.40828620959051, 78.66950509876006, 400 },
		{ -74.43215130380513, 45.4087793830007, 75.92158384183497, 400 },
		{ -74.43156604844955, 45.40865767306189, 42.30715473879556, 400 },
		{ -74.43222539295675, 45.40644700649151, 101.89416019653214, 400 },
		{ -74.431690, 45.407910, 60, 200 }
	};*/

	vector<coordinatesObst> obstacles = {
		{ -74.432349, 45.409207, 40, 200 },
		{ -74.431940, 45.409132, 40, 200 },
		{ -74.431449, 45.409064, 40, 200 },
		{ -74.432197, 45.408874, 40, 200 },
		{ -74.431780, 45.408702, 40, 200 },
		{ -74.432545, 45.408610, 40, 200 },
		{ -74.432159, 45.408502, 40, 200 },
		{ -74.431730, 45.408469, 40, 200 },
		{ -74.432454, 45.408436, 40, 200 },
		{ -74.432131, 45.408422, 40, 200 },
		{ -74.431851, 45.408268, 40, 200 },
		{ -74.432770, 45.408151, 40, 200 },
		{ -74.432331, 45.408057, 40, 200 },
		{ -74.431906, 45.407983, 40, 200 },
		{ -74.432774, 45.407864, 40, 200 },
		{ -74.432118, 45.407746, 40, 200 },
		{ -74.432900, 45.407651, 40, 200 },
		{ -74.432465, 45.407569, 40, 200 },
		{ -74.432121, 45.407364, 40, 200 },
		{ -74.432990, 45.407314, 40, 200 },
		{ -74.432377, 45.407253, 40, 200 },
		{ -74.432916, 45.407123, 40, 200 },
		{ -74.432400, 45.407033, 40, 200 },
		{ -74.432903, 45.406731, 40, 200 },
		{ -74.432621, 45.406414, 40, 200 },
		{ -74.433327, 45.406440, 40, 200 },
		{ -74.433009, 45.406236, 40, 200 }
	};

	// CONFIG CHALET HALF FLY ZONE
	/*vector<coordinates> flyZones = {
		{ -74.4334263491007, 45.40686496264888, 0 },{ -74.43205360837723, 45.40662283778793, 0 },{ -74.43235160561015, 45.40591878920772, 0 },{ -74.43369421178144, 45.40617267084293, 0 }
	};

	vector<coordinates> waypoints = {
		{ -74.432464747462, 45.406116221166826, 150 },{ -74.43234273248224, 45.40642570353321, 150 },{ -74.43327458871224, 45.40642231832365, 250 }, { -74.43285429705311, 45.406546969949595, 250},{ -74.4331303711341, 45.40622842383921, 350 },{ -74.4327294635466, 45.406238536433506, 350 },{ -74.432836, 45.406025, 150 }
	};

	vector<coordinatesObst> obstacles = {
		{ -74.432464747462, 45.406116221166826, 16, 100 }, { -74.43234273248224, 45.40642570353321, 18, 100 },{ -74.43327458871224, 45.40642231832365, 35, 200 },{ -74.43285429705311, 45.406546969949595, 60, 200 },{ -74.4331303711341, 45.40622842383921, 28, 300 },{ -74.4327294635466, 45.406238536433506, 32, 300 }
	};*/

	// Altitude
	double altMin = 100; // feet MSL
	double altMax = 750;

	// Obstacle radius
	double radMin = 30; // feet
	double radMax = 300;

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

	// Get random configuration
	/*vector<coordinates> waypoints;
	vector<coordinatesObst> obstacles;
	tie(waypoints, obstacles) = getRandomConfig(Ymin, Xmin, Ymax, Xmax, altMin, altMax, radMin, radMax);*/
	
	// Resolution & safety distance
	double resolution = 1; // m
	double safetyDist = 3; // m

	// If a waypoint is above an obstacle
	double d = 0, newLat = 0, newLon = 0;
	coordinates newPoint, obstCenter, obstPerimeter;
	double wSize = waypoints.size();
	for (int i = 0; i < waypoints.size(); ++i)
	{
		// Check if a waypoint is inside an obstacle
		for (int j = 0; j < obstacles.size(); ++j)
		{
			if (obstacles[j].height < waypoints[i].altitude)
			{
				d = distanceEarth(waypoints[i].latitude, waypoints[i].longitude, obstacles[j].latitude, obstacles[j].longitude);

				// Waypoint inside
				if (d*feet2meter < (obstacles[j].radius + safetyDist*feet2meter))
				{

					if (i != waypoints.size() - 1)
					{
						cout << "d : " << obstacles[j].radius - d * feet2meter << endl;
						cout << "r : " << obstacles[j].radius << endl;

						obstCenter = { obstacles[j].longitude, obstacles[j].latitude, obstacles[j].height };
						newPoint = GenerateCirclePoint(obstacles[j], waypoints[i], getBearing2Points(waypoints[i], waypoints[i + 1]), safetyDist, 0);

						//newPoint = GenerateCirclePoint(obstacles[j], waypoints[i], getBearing2Points(waypoints[i], waypoints[i+1]), safetyDist, obstacles[j].radius - d*feet2meter);
						waypoints.insert(waypoints.begin() + i + 1, newPoint);
						i++;

						if (i != 1)
						{
							//newPoint = GenerateCirclePoint(obstacles[j], waypoints[i - 1], getBearing2Points(waypoints[i - 1], waypoints[i - 2]), safetyDist, obstacles[j].radius - d * feet2meter);
							newPoint = GenerateCirclePoint(obstacles[j], waypoints[i - 1], getBearing2Points(waypoints[i - 1], waypoints[i - 2]), safetyDist, 0);
							waypoints.insert(waypoints.begin() + i - 1, newPoint);
							i++;
						}
					}
				}
			}
		}
	}


	/*cout.precision(8);
	cout << flyZones[index_Xmax].longitude << " " << flyZones[index_Xmax].latitude << " Xmax " << Xmax << endl;
	cout << flyZones[index_Ymax].longitude << " " << flyZones[index_Ymax].latitude << " Ymax " << Ymax << endl;
	cout << flyZones[index_Xmin].longitude << " " << flyZones[index_Xmin].latitude << " Xmin " << Xmin << endl;
	cout << flyZones[index_Ymin].longitude << " " << flyZones[index_Ymin].latitude << " Ymin " << Ymin << endl;*/

	/*double limitsX = distanceEarth(flyZones[index_Xmin].latitude, flyZones[index_Xmin].longitude, flyZones[index_Xmax].latitude, flyZones[index_Xmax].longitude);
	double limitsY = distanceEarth(flyZones[index_Ymin].latitude, flyZones[index_Ymin].longitude, flyZones[index_Ymax].latitude, flyZones[index_Ymax].longitude);*/

	double limitsX = distanceEarth(Ymin, Xmin, Ymin, Xmax);
	double limitsY = distanceEarth(Ymin, Xmin, Ymax, Xmin);
	//cout << limitsX << " " << limitsY << endl;

	coordinates center = { (Xmax + Xmin) / 2, (Ymax + Ymin) / 2, 0 };
	//cout << "center : " << center.longitude << " " << center.latitude << endl;

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

	/*for (int i = 0; i < startStates.size(); ++i)
	{
		cout << startStates[i].x << " " << startStates[i].y << " **** " << goalStates[i].x << " " << goalStates[i].y << endl;
	}*/

	// SAVE FLY ZONE GPS IN FILE
	ofstream flyZoneGpsFile(FLY_ZONE_GPS_FILE, std::ios_base::app);
	flyZoneGpsFile << setprecision(15);

	for (int i = 0; i < flyZones.size(); ++i)
	{
		flyZoneGpsFile << flyZones[i].latitude << "," << flyZones[i].longitude << endl;
	}
	
	// SAVE INITIAL GPS OBSTACLES IN FILE
	ofstream obstacleInitialGpsFile(OBSTACLES_INITIAL_GPS_FILE, std::ios_base::app);
	obstacleInitialGpsFile << setprecision(15);

	for (int i = 0; i < obstacles.size(); ++i)
	{
		obstacleInitialGpsFile << obstacles[i].latitude << "," << obstacles[i].longitude << "," << obstacles[i].radius * meter2feet << "," << obstacles[i].height << endl;
	}

	// SAVE CARTESIAN OBSTACLES IN FILE
	ofstream obstacleCartesianFile(OBSTACLES_CARTESIAN_FILE, std::ios_base::app);
	obstacleCartesianFile << setprecision(15);

	for (int i = 0; i < obstacles.size(); ++i)
	{
		coordinates cylinder = coordinates{ obstacles[i].longitude, obstacles[i].latitude, obstacles[i].height };
		point point = toCartesianPoint(cylinder, center, limitsX, limitsY, Xmax, Ymax);
		double radius = feet2met(obstacles[i].radius);

		obstacleCartesianFile << point.x << "," << point.y << "," << point.z << "," << radius << endl;
	}

	// OBSTACLES TO POLYGONS
	ofstream obstacleGpsFile(OBSTACLES_GPS_FILE, std::ios_base::app);
	obstacleGpsFile << setprecision(15);

	vector< vector<point> > obstaclesArray;
	for (int i = 0; i < obstacles.size(); ++i)
	{
		vector<coordinates> obstacle2Polygon = GenerateCirclePolygon(obstacles[i]);

		for (int i = 0; i < obstacle2Polygon.size(); ++i)
		{
			obstacleGpsFile << setprecision(15);
			obstacleGpsFile << obstacle2Polygon[i].latitude << "," << obstacle2Polygon[i].longitude << endl;

			//cout << obstacle2Polygon[i].latitude << "," << obstacle2Polygon[i].longitude << endl;
		}

		vector<point> newObstaclePolygon = vecPoint2vecCoord(obstacle2Polygon, center, limitsX, limitsY, Xmax, Ymax);
		obstaclesArray.push_back(newObstaclePolygon);
	}

	// FLY ZONE TO POLYGON
	vector<point> flyZonePolygon = vecPoint2vecCoord(flyZones, center, limitsX, limitsY, Xmax, Ymax);
	vector<coordinates> flyZoneRectangleGPS = { {Xmin, Ymin}, {Xmax, Ymin}, {Xmax, Ymax}, {Xmin, Ymax}, {Xmin, Ymin} };
	vector<point> flyZoneRectangle = vecPoint2vecCoord(flyZoneRectangleGPS, center, limitsX, limitsY, Xmax, Ymax);

	// Bearing
	for (int i = 0; i < startStates.size(); ++i)
		computeAngle2Points(startStates[i], goalStates[i]);

	PathPlanner<HybridAlgorithm> pathPlanner(startStates[0], goalStates[0], obstaclesArray, limitsX, limitsY, resolution, safetyDist);
	pathPlanner.loadFlyZone(flyZonePolygon, flyZoneRectangle);

	pathPlanner.setSearchAlgorithm("AStarHybrid");
	//pathPlanner.setSearchAlgorithmHeuristic(grid::MANHATTAN);

	pathPlanner.getSearchAlgorithm().setVelocity(1); // 100
	pathPlanner.getSearchAlgorithm().setAngularVelocity(0.5); // 0.25
	pathPlanner.getSearchAlgorithm().setDubinsTurnRadius(0.1); // 1
	//pathPlanner.getSearchAlgorithm().doNotUseDubins();
	
	// Next path planners
	for (int i = 0; i < startStates.size(); ++i)
	{
		try
		{
			if (i > 0)
			{
				pathPlanner.setStartState(startStates[i]);
				pathPlanner.setGoalState(goalStates[i]);
			}

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
				waypointsGpsFile << setprecision(15);

				for (int i = 0; i < path.size(); ++i)
				{
					coordinates coord = toCoordinates(path[i], center, limitsX, limitsY, Xmax, Ymax);
					waypointsGpsFile << coord.latitude << "," << coord.longitude << "," << coord.altitude * meter2feet << endl;
				}

				if (i == startStates.size() - 1)
				{
					coordinates coord = toCoordinates(goalStates[i], center, limitsX, limitsY, Xmax, Ymax);
					waypointsGpsFile << coord.latitude << "," << coord.longitude << "," << coord.altitude * meter2feet << endl;
				}

				/*if (i == 0)
				{
					coordinates coord = toCoordinates(startStates[i], center, limitsX, limitsY, Xmax, Ymax);
					waypointsGpsFile << coord.latitude << "," << coord.longitude << "," << coord.altitude * meter2feet << endl;
				}*/
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
  pathPlannerWaypoints();

  return 0;
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