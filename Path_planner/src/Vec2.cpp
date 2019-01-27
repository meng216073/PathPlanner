//  Vec2.cpp
//    2D vector type implementation.

#include "Vec2.h"
#include <cmath>

Vec2::Vec2()
{
    mX = 0;
    mY = 0;
}

Vec2::Vec2(double val)
{
    mX = val;
    mY = val;
}

Vec2::Vec2(double _x, double _y)
{
    mX = _x;
    mY = _y;
}

Vec2::Vec2(const Vec2& v)
{
    mX = v.mX;
    mY = v.mY;
}

Vec2::~Vec2()
{ }

double Vec2::x() const
{
    return mX;
}

double Vec2::y() const
{
    return mY;
}

double Vec2::norm() const
{
    return std::sqrt(mX*mX + mY*mY);
}

void Vec2::setX(double val)
{
    mX = val;
}

void Vec2::setY(double val)
{
    mY = val;
}

void Vec2::set(double _x, double _y)
{
    mX = _x;
    mY = _y;
}

Vec2 Vec2::rotate(double angle)
{
    double cs = std::cos(angle);
    double sn = std::sin(angle);
    double x = mX*cs + mY*sn;
    double y = -mX*sn + mY*cs;
    Vec2 v(x,y);
    return v;
}

double Vec2::dot(Vec2 vec)
{
  return mX * vec.x() + mY * vec.y();
}

Vec2 Vec2::ort(Vec2 vec)
{
  return *this - (vec * this->dot(vec)) / (pow(vec.x(), 2) + pow(vec.y(), 2)); // DELETE POW
}

double Vec2::angle() const
{
    return std::atan2(mY, mX);
}

Vec2 Vec2::symLimit(double limit)
{
    double x = std::fmax(-limit, std::fmin(limit, mX));
    double y = std::fmax(-limit, std::fmin(limit, mY));
    Vec2 v(x,y);
    return v;
}

Vec2 Vec2::unit() const
{
    return unit(1.0e-10);
}

Vec2 Vec2::unit(double eps) const
{
    double a = norm();
    if (a < eps)
    {
        return Vec2(0.0, 0.0);
    }
    else
    {
        // use argument division for greater numeric precision than
        // vector division (*this / a) when components are near 1.0
        //
        return Vec2(mX/a, mY/a);
    }    
}


Vec2 Vec2::operator= (double a)
{
    mX = a;
    mY = a;
    return (*this);
}

bool Vec2::operator> (double a)
{
    return (this->norm() > a);
}

bool Vec2::operator< (double a)
{
    return !(*this > a);
}

Vec2 operator+ (const Vec2& left, const Vec2& right)
{
    double x = left.x() + right.x();
    double y = left.y() + right.y();
    Vec2 v(x,y);
    return v;
}

Vec2 operator- (const Vec2& right)
{
    Vec2 v(-right.x(), -right.y());
    return v;
}

Vec2 operator- (const Vec2& left, const Vec2& right)
{
    double x = left.x() - right.x();
    double y = left.y() - right.y();
    Vec2 v(x,y);
    return v;
}

Vec2 operator* (const Vec2& left, double a)
{
    double x = left.x() * a;
    double y = left.y() * a;
    Vec2 r(x,y);
    return r;
}

Vec2 operator/ (const Vec2& left, double a)
{
    return left * (1.0 / a);
}


Vec2& operator+= (Vec2& left, const Vec2& right)
{
    left = left + right;
	return left;
}

Vec2& operator-= (Vec2& left, const Vec2& right)
{
    left = left - right;
	return left;
}

Vec2& operator*= (Vec2& v, double s)
{
    v = v * s;
	return v;
}

Vec2& operator/= (Vec2& v, double s)
{
    v *= (1.0 / s);
	return v;
}

std::ostream& operator<< (std::ostream& stream, const Vec2& v)
{
    stream << "(" << v.x() << ", " << v.y() << ")";
    return stream;
}
