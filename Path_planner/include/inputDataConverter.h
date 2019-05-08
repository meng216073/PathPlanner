#pragma once

#define _USE_MATH_DEFINES

#define R_earth 6371 // km
#define feet2meter 3.2808399
#define meter2feet 0.3048

#include <cmath>
#include <iostream>

#include "searchgrid.h"


// This function converts decimal degrees to radians
double deg2rad(double deg) {
	return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
	return (rad * 180 / M_PI);
}

double feet2met(double feet)
{
	return feet * meter2feet;
}

// longitude : degrees
// latitude : degrees
// altitude : feet
// output : meters
state toCartesianState(coordinates waypoint, coordinates center, double limitsX, double limitsY, double Xmax, double Ymax)
{
	double x = (center.longitude - waypoint.longitude) * (limitsX / 2) / (center.longitude - Xmax);
	double y = (center.latitude -  waypoint.latitude) * (limitsY / 2) / (center.latitude - Ymax);
	double z = meter2feet * waypoint.altitude;

	return state{ x, y, 0, z};
}

point toCartesianPoint(coordinates waypoint, coordinates center, double limitsX, double limitsY, double Xmax, double Ymax)
{
	double x = (center.longitude - waypoint.longitude) * (limitsX / 2) / (center.longitude - Xmax);
	double y = (center.latitude - waypoint.latitude) * (limitsY / 2) / (center.latitude - Ymax);
	double z = meter2feet * waypoint.altitude;

	return point{ x, y, z };
}

coordinates toCoordinates(state waypoint, coordinates center, double limitsX, double limitsY, double Xmax, double Ymax)
{
	double lon = center.longitude - (2 * waypoint.x * (center.longitude - Xmax))/ limitsX;
	double lat = center.latitude - (2 * waypoint.y * (center.latitude - Ymax)) / limitsY;
	double alt = feet2meter * waypoint.z;

	return coordinates{ lon, lat, alt };
}

coordinates toCoordinatesPoint(point waypoint, coordinates center, double limitsX, double limitsY, double Xmax, double Ymax)
{
	double lon = center.longitude - (2 * waypoint.x * (center.longitude - Xmax)) / limitsX;
	double lat = center.latitude - (2 * waypoint.y * (center.latitude - Ymax)) / limitsY;
	double alt = feet2meter * waypoint.z;

	return coordinates{ lon, lat, alt };
}

std::vector<point> vecPoint2vecCoord(const std::vector<coordinates>& vecCoord, coordinates center, double limitsX, double limitsY, double Xmax, double Ymax)
{
	std::vector<point> vecPoint;

	for (int i = 0; i < vecCoord.size(); ++i)
	{
		vecPoint.push_back(toCartesianPoint(vecCoord[i], center, limitsX, limitsY, Xmax, Ymax));
	}

	return vecPoint;
}

/**
* Returns the distance between two points on the Earth. (Haversine formula)
* @param lat1d Latitude of the first point in degrees
* @param lon1d Longitude of the first point in degrees
* @param lat2d Latitude of the second point in degrees
* @param lon2d Longitude of the second point in degrees
* @return The distance between the two points in meters
*/
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) 
{
	double lat1r, lon1r, lat2r, lon2r, u, v;
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = sin((lat2r - lat1r) / 2);
	v = sin((lon2r - lon1r) / 2);
	return 2.0 * R_earth * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)) * 1000;
}

void computeAngle2Points(state& pt1, state& pt2)
{
	if (pt1.x == pt2.x && pt1.y == pt2.y)
		return;

	double theta = atan2(pt2.y - pt1.y, pt2.x - pt1.x);
	if (theta < 0.0)
		theta += 2*M_PI;

	pt1.theta = theta;
	pt2.theta = theta;
}

// GenerateCirlcePolygon - Creates Circle from 360 line Segments
//*@param center coordinates in degrees
//* @param radius in feet
std::vector<coordinates> GenerateCirclePolygon(coordinatesObst obstacle)
{
	//latitude in radians
	double lat = deg2rad(obstacle.latitude);
		
	//longitude in radians
	double lon = deg2rad(obstacle.longitude);

	//angular distance covered on earth's surface
	double d = ((meter2feet / 1000) * obstacle.radius) / R_earth;
	std::vector<coordinates> circlePolygon;

	for (int i = 0; i < 360; i++) 
	{
		coordinates point;
		double bearing = deg2rad(i); //rad

		point.latitude = asin(sin(lat) * cos(d) + cos(lat) * sin(d) * cos(bearing));
		point.longitude = rad2deg(lon + atan2(sin(bearing) * sin(d) * cos(lat), cos(d) - sin(lat) * sin(point.latitude)));
		point.latitude = rad2deg(point.latitude);

		circlePolygon.push_back(point);
	}

	return circlePolygon;
}