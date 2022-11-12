#ifndef CIRCLE_H
#define CIRCLE_H

#include "Vector.h"
#include "Line.h"
#include "Ray.h"

/**
 * 圆类
 */
class Circle {
public:
  Circle(const Vector &center = Vector(0.0, 0.0),
                  const double radius = 0.0)
      : center_(center), radius_(radius) {}
  Circle(const double center_x, const double center_y, const double radius)
      : center_(Vector(center_x, center_y)), radius_(radius) {}
  Circle(const Vector &point1, const Vector &point2, const Vector &point3) {
    Line l1 = Line::GetCentralPerpendicularLine(point1, point2);
    Line l2 = Line::GetCentralPerpendicularLine(point2, point3);

    if (l1.Intersection(l2, center_) == false) {
      center_ = Vector(0.0, 0.0);
    }
    radius_ = center_.Dist(point1);
  }

  const Vector &Center() const { return center_; }
  const double Radius() const { return radius_; }
  void SetCenter(const Vector &center) { center_ = center; }
  void SetRadius(const double radius) { radius_ = radius; }

  bool IsWithin(const Vector &p, const double buffer = FLOAT_EPS) {
    return center_.Dist(p) <= radius_ + buffer;
  }

  /**
   * Get intersection points between the circle and the ray.
   * t1 is nearer to the origin than t2.
   * \param r the ray.
   * \param t1 will be set to the distance from origin of the ray to
   * intersection point 1. \param t2 will be set to the distance from origin of
   * the ray to intersection point 2. \param buffer controls precision. \return
   * number of intersection points.
   */
  int Intersection(const Ray &r, double t1, double t2,
                   const double buffer = 0.0) const;
  int Intersection(const Circle &c, Vector &v1, Vector &v2,
                   const double buffer) const;

private:
  Vector center_;
  double radius_;
};
#endif