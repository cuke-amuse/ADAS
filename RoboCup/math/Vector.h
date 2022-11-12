#ifndef VECTOR_H
#define VECTOR_H

/**
 * 向量类
 */
class Vector {
public:
  Vector(const double x = 0.0, const double y = 0.0)
   : mX(x), 
     mY(y) {
    //AssertValid();
  }

  const double X() const { return mX; }   // shouble be value,  reference may degree the effeciect
  const double Y() const { return mY; }

  void SetX(const double x) {
    mX = x;
    //Assert(!IsInvalid(mX));
  }
  void SetY(const double y) {
    mY = y;
    //Assert(!IsInvalid(mY));
  }

  void SetValue(const double x, const double y) {
    mX = x;
    mY = y;
    //AssertValid();
  }
  void SetValuePolar(const double r, const AngleDeg &theta) {
    SinCosT value = SinCos(theta);

    mX = r * Cos(value);
    mY = r * Sin(value);

    AssertValid();
  }

  Vector operator-() const { return Vector(-mX, -mY); }   // should return objects c = a- b
  Vector operator+(const Vector &a) const {
    return Vector(mX + a.mX, mY + a.mY);
  }
  Vector operator-(const Vector &a) const {
    return Vector(mX - a.mX, mY - a.mY);
  }
  Vector operator*(const double a) const { return Vector(mX * a, mY * a); }
  Vector operator/(double a) const {
    if (a > -FLOAT_EPS && a < FLOAT_EPS) {
      a = Sign(a) * FLOAT_EPS;
    }
    return Vector(mX / a, mY / a);
  }

  void operator+=(const Vector &a) {   //  check later
    mX += a.mX;
    mY += a.mY;
    AssertValid();
  }
  void operator+=(const double a) {
    mX += a;
    mY += a;
    AssertValid();
  }
  void operator-=(const Vector &a) {
    mX -= a.mX;
    mY -= a.mY;
    AssertValid();
  }
  void operator-=(const double a) {
    mX -= a;
    mY -= a;
    AssertValid();
  }
  void operator*=(const double a) {
    mX *= a;
    mY *= a;
    AssertValid();
  }
  void operator/=(const double a) {
    mX /= a;
    mY /= a;
    AssertValid();
  }

  bool operator!=(const Vector &a) const {
    return (mX != a.mX) || (mY != a.mY);
  }
  bool operator!=(const double a) const { return (mX != a) || (mY != a); }
  bool operator==(const Vector &a) const {
    return (mX == a.mX) && (mY == a.mY);
  }

  friend std::ostream &operator<<(std::ostream &os, const Vector &v) {
    return os << "(" << v.mX << ", " << v.mY << ")";
  }

  double Mod() const { return Sqrt(mX * mX + mY * mY); }
  double Mod2() const { return mX * mX + mY * mY; }
  double Dist(const Vector &a) const { return (*this - a).Mod(); }
  double Dist2(const Vector &a) const { return (*this - a).Mod2(); }

  AngleDeg Dir() const { return ATan2(mY, mX); }

  /**
   * \return a Vector with length "length" at the same direction, or Vector (0,
   * 0) if the original Vector was (0, 0).
   */
  Vector SetLength(const double length) const {
    if (Mod() > 0.0) {
      return (*this) * (length / Mod());
    }
    return Vector(0.0, 0.0);
  }

  /**
   * \return a Vector with length 1.0 at the same direction.
   */
  Vector Normalize() const { return SetLength(1.0); }

  /**
   * \return a Vector rotated by angle.
   */
  Vector Rotate(const AngleDeg &angle) const { return Rotate(SinCos(angle)); }

  Vector Rotate(const SinCosT &value) const {
    return Vector(mX * Cos(value) - mY * Sin(value),
                  mY * Cos(value) + mX * Sin(value));
  }

  /**
   * check if a point is approximate equal to *this;
   * @param point to be checked.
   * return true when they are approximate equal, false else;
   */
  bool ApproxEqual(const Vector &a) const {
    return fabs(mX - a.X()) < FLOAT_EPS && fabs(mY - a.Y()) < FLOAT_EPS;
  }

private:
  void AssertValid() {
    Assert(!IsInvalid(mX));
    Assert(!IsInvalid(mY));
  }

private:
  double mX;
  double mY;
};

inline Vector Polar2Vector(const double mod, const AngleDeg &ang) {
  SinCosT value = SinCos(ang);

  return Vector(mod * Cos(value), mod * Sin(value));
}

#endif