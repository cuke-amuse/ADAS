#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "Vector.h"
#include "Ray.h"

/**
 * 矩形类
 */
class Rectangular {
public:
  Rectangular() : mLeft(0.0), mRight(0.0), mTop(0.0), mBottom(0.0) {}
  Rectangular(const double left, const double right, const double top,
              const double bottom)
      : mLeft(left), mRight(right), mTop(top), mBottom(bottom) {}

  Rectangular(const Vector &center, const Vector &size) {
    mLeft = center.X() - size.X() / 2.0;
    mRight = center.X() + size.X() / 2.0;
    mTop = center.Y() - size.Y() / 2.0;
    mBottom = center.Y() + size.Y() / 2.0;
  }

  const double Left() const { return mLeft; }
  const double Right() const { return mRight; }
  const double Top() const { return mTop; }
  const double Bottom() const { return mBottom; }

  void SetLeft(const double left) { mLeft = left; }
  void SetRight(const double right) { mRight = right; }
  void SetTop(const double top) { mTop = top; }
  void SetBottom(const double bottom) { mBottom = bottom; }

  Vector TopLeftCorner() const { return Vector(mLeft, mTop); }
  Vector TopRightCorner() const { return Vector(mRight, mTop); }
  Vector BottomLeftCorner() const { return Vector(mLeft, mBottom); }
  Vector BottomRightCorner() const { return Vector(mRight, mBottom); }

  Line TopEdge() const { return Line(TopLeftCorner(), TopRightCorner()); }
  Line BottomEdge() const {
    return Line(BottomLeftCorner(), BottomRightCorner());
  }
  Line LeftEdge() const { return Line(TopLeftCorner(), BottomLeftCorner()); }
  Line RightEdge() const { return Line(TopRightCorner(), BottomRightCorner()); }

  bool IsWithin(const Vector &v, const double buffer = FLOAT_EPS) const {
    return (v.X() >= mLeft - buffer) && (v.X() <= mRight + buffer) &&
           (v.Y() >= mTop - buffer) && (v.Y() <= mBottom + buffer);
  }

  bool Intersection(const Ray &r, Vector &point) const;
  Vector Intersection(const Ray &r) const;

  Vector AdjustToWithin(const Vector &v) const {
    Vector r = v;

    if (r.X() < mLeft) {
      r.SetX(mLeft);
    } else if (r.X() > mRight) {
      r.SetX(mRight);
    }

    if (r.Y() < mTop) {
      r.SetY(mTop);
    } else if (r.Y() > mBottom) {
      r.SetY(mBottom);
    }

    return r;
  }

private:
  double mLeft;   // 矩形左边
  double mRight;  // 矩形右边
  double mTop;    // 矩形上边
  double mBottom; // 矩形下边
};

#endif