#ifndef UTIL_H_
#define UTIL_H_

#define _USE_MATH_DEFINES
#include <vector>

// Fold angle in radians to +/-pi and 0-2pi
double foldOverPi(double angle);
double foldOverTwoPi(double angle);

// Clamp a number between a min and max value 
double clamp(double n, double lower, double upper);

// Linespace
std::vector<double> linspace(double a, double b, std::size_t N);

#endif
