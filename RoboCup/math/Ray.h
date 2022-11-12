#ifndef RAY_H
#define RAY_H

#include "Vector.h"

/**
 * 射线类
 */
class Line;

class Ray {
public:
  Ray() {}

  explicit Ray(const Vector &origin, const AngleDeg &direction) {
    SetValue(origin, direction);
  }

  const Vector &Origin() const { return mOrigin; }
  const AngleDeg &Dir() const { return mDirection; }

  void SetOrigin(const Vector &origin) { mOrigin = origin; }
  void SetDirection(const AngleDeg &direction) { mDirection = direction; }
  void SetValue(const Vector &origin, const AngleDeg &direction) {
    mOrigin = origin;
    mDirection = direction;
  }

  Vector GetPoint(const double dist) const {
    return mOrigin + Polar2Vector(dist, mDirection);
  }

  bool IsInRightDir(const Vector &point) const {
    return fabs(GetNormalizeAngleDeg((point - mOrigin).Dir() - mDirection)) <
           10.0;
  }

  bool OnRay(const Vector &point, const double buffer = FLOAT_EPS) const {
    Vector v = point - mOrigin;
    return fabs(Sin(v.Dir() - mDirection) * v.Mod()) < buffer &&
           IsInRightDir(point);
  }

  bool Intersection(const Line &l, double intersection_dist) const;
  bool Intersection(const Line &l, Vector &point) const;
  bool Intersection(const Ray &r, Vector &point) const;
  bool Intersection(const Ray &r, double intersection_dist) const;
  double Intersection(const Line &l) const;

  /*得到一条射线上离这个点最近的点*/
  Vector
  GetClosestPoint(const Vector &point) const; // add by wang yu hang -09.1.14

  inline double GetDistanceFromOrigin(const Vector &point) const {
    return (point - mOrigin).Mod();
  }

private:
  Vector mOrigin;
  AngleDeg mDirection;
};

#endif