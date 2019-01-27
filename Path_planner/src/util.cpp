#include "util.h"
#include <cmath>

double foldOverPi(double angle)
{
  double result = angle;
  while (result > M_PI)
  {
    result -= 2 * M_PI;
  }
  while (result < -M_PI)
  {
    result += 2 * M_PI;
  }
  return result;
}

double foldOverTwoPi(double angle)
{
  double result = angle;
  while (result > 2 * M_PI)
  {
    result -= 2 * M_PI;
  }
  while (result < 0.0)
  {
    result += 2 * M_PI;
  }  
  return result;
}

double clamp(double n, double lower, double upper)
{
  return fmax(lower, fmin(n, upper));
}

std::vector<double> linspace(double a, double b, std::size_t N)
{
  double h = (b - a) / static_cast<double>(N-1);
  std::vector<double> xs(N);
  typename std::vector<double>::iterator it;
  double val;

  for (it = xs.begin(), val = a; it != xs.end(); ++it, val += h) 
  {
    *it = val;
  }

  return xs;
}