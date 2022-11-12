#ifndef LINE_H
#define LINE_H

#include "Vector.h"
#include "Ray.h"
/**
 * 直线类
 */
class Line {
public:
  /** a*x+b*y+c=0 */
  explicit Line(const double a = 0.0, const double b = 0.0,
                const double c = 0.0)
      : mA(a), mB(b), mC(c) {}

  Line(const Vector &point1, const Vector &point2) {
    if (fabs(point1.X() - point2.X()) < FLOAT_EPS) {
      mA = 1.0;
      mB = 0.0;
      mC = -point1.X();
    } else {
      mA = (point2.Y() - point1.Y()) / (point2.X() - point1.X());
      mB = -1.0;
      mC = point1.Y() - point1.X() * mA;
    }
  }

  Line(const Ray &r) {
    *this = Line(r.Origin(), r.Origin() + Polar2Vector(1.0, r.Dir()));
  }

  const double A() const { return mA; }
  const double B() const { return mB; }
  const double C() const { return mC; }
  double Dir() const { return ATan(-mA / mB); }

  double GetX(const double y) const {
    if (fabs(mA) > 0.0) {
      return ((-mC - mB * y) / mA);
    } else {
      return 0.0;
    }
  }

  double GetY(const double x) const {
    if (fabs(mB) > 0.0) {
      return ((-mC - mA * x) / mB);
    } else {
      return 0.0;
    }
  }

  /** 是否在直线上 */
  bool IsOnLine(const Vector &point, const double buffer = FLOAT_EPS) const {
    return fabs(mA * point.X() + mB * point.Y() + mC) < buffer;
  }

  /** 是否在直线上方 */
  bool IsUpLine(const Vector &point) const {
    return !IsOnLine(point) && (mA * point.X() + mB * point.Y() + mC > 0);
  }

  bool HalfPlaneTest(const Vector &pt) {
    if (fabs(mB) > 0.0) {
      return pt.Y() > GetY(pt.X());
    }
    return pt.X() < -mC / mA;
  }

  /** 斜率是否相等 */
  bool IsSameSlope(const Line &l, const double buffer = FLOAT_EPS) const {
    return (fabs(mB) < buffer && fabs(l.mB) < buffer) ||
           fabs(mA / mB - l.mA / l.mB) < buffer;
  }

  /**
   * 判断一点的垂足是否在两点之间
   */
  bool IsInBetween(const Vector &pt, const Vector &end1,
                   const Vector &end2) const;

  /** 求交点 */
  bool Intersection(const Line &l, Vector &point) const;
  Vector Intersection(const Line &l) const;

  /** 求与射线的交点 */
  bool Intersection(const Ray &r, Vector &point) const;

  /** 点到直线的距离 */
  double Dist(const Vector &point) const {
    return fabs(mA * point.X() + mB * point.Y() + mC) / Sqrt(mA * mA + mB * mB);
  }

  /** 两点是否在直线同侧 */
  bool IsPointInSameSide(const Vector &pt1, const Vector &pt2) const {
    Line tl(pt1, pt2);
    if (IsSameSlope(tl))
      return true;

    Vector inter_point;
    Intersection(tl, inter_point);

    return (inter_point.X() - pt1.X()) * (pt2.X() - inter_point.X()) <= 0;
  }

  /** 过pt点的垂线 **/
  Line GetPerpendicular(const Vector &pt) const {
    return Line(mB, -mA, mA * pt.Y() - mB * pt.X());
  }

  /**
   * Set this line be the perpendicular bisector of pos1 and pos2;
   * @param one point;
   * @param another point;
   */
  void PerpendicularBisector(const Vector &pos1, const Vector &pos2) {
    mA = 2 * (pos2.X() - pos1.X());
    mB = 2 * (pos2.Y() - pos1.Y());
    mC = pos1.X() * pos1.X() - pos2.X() * pos2.X() + pos1.Y() * pos1.Y() -
         pos2.Y() * pos2.Y();
  }

  /** 得到投影点 */
  Vector GetProjectPoint(const Vector &pt) const {
    Vector joint_pt;
    Intersection(GetPerpendicular(pt), joint_pt);
    return joint_pt;
  }

  //得到对称点
  inline Vector MirrorPoint(const Vector &pt) {
    return GetProjectPoint(pt) * 2.0 - pt;
  }

  /**
   * 得到直线上两点间距离这个点最近的点
   */
  Vector GetClosestPointInBetween(const Vector &pt, const Vector &end1,
                                  const Vector &end2) const;

  void LineFromPline(const Vector &pos1, const Vector &pos2) {
    mA = 2 * (pos2.X() - pos1.X());
    mB = 2 * (pos2.Y() - pos1.Y());
    mC = pos1.X() * pos1.X() - pos2.X() * pos2.X() + pos1.Y() * pos1.Y() -
         pos2.Y() * pos2.Y();
  }

  const double GetA() const { return mA; }
  const double GetB() const { return mB; }
  const double GetC() const { return mC; }

/** Get the central perpendicular line from two points */
inline static Line GetCentralPerpendicularLine(const Vector &pos1,
                                        const Vector &pos2) {
  double a = 2.0 * (pos2.X() - pos1.X());
  double b = 2.0 * (pos2.Y() - pos1.Y());
  double c = pos1.X() * pos1.X() - pos2.X() * pos2.X() + pos1.Y() * pos1.Y() -
             pos2.Y() * pos2.Y();
  return Line(a, b, c);
}

private:
  /**
   * Ax + By + C = 0
   */
  double mA;
  double mB;
  double mC;
};

#endif