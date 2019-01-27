//  Vec2.h
//    2D vector type declaration.

#ifndef _PRT_MATH_VEC2_H_
#define _PRT_MATH_VEC2_H_

#define _USE_MATH_DEFINES
#include <ostream>

class Vec2
{
public:
  Vec2();
  Vec2(double val);
  Vec2(double _x, double _y);
  Vec2(const Vec2& v);

  virtual ~Vec2();

  // compare vector components
  Vec2 operator= (double a);

  // compare vector norm
  bool operator> (double a);
  bool operator< (double a);

  double x() const;
  double y() const;
  double norm() const;

  void setX(double val);
  void setY(double val);
  void set(double _x, double _y);

  // return new vector rotated by angle in radians
  Vec2 rotate(double angle);

  // dot product
  double dot(Vec2 vec);

  // orthogonal complement
  Vec2 ort(Vec2 vec);

  // angle from x-axis, +/-pi radians
  double angle() const;

  // apply a symmetric limit to x and y components 
  Vec2 symLimit(double limit);

  // return unit vector parallel to this vector or
  // zero vector if norm is less than 'eps' argument,
  // default eps = 1e-10.
  Vec2 unit(double eps) const;
  Vec2 unit() const;

private:
  double mX;
  double mY;
};

Vec2 operator+ (const Vec2& left, const Vec2& right);
Vec2 operator- (const Vec2& right);
Vec2 operator- (const Vec2& left, const Vec2& right);
Vec2 operator* (const Vec2& left, double a);
Vec2 operator/ (const Vec2& left, double a);
Vec2& operator+= (Vec2& left, const Vec2& right);
Vec2& operator-= (Vec2& left, const Vec2& right);
Vec2& operator*= (Vec2& v, double s);
Vec2& operator/= (Vec2& v, double s);

std::ostream& operator<< (std::ostream& stream, const Vec2& v);

#endif
