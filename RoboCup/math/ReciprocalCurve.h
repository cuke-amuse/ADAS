#ifndef RECIPROCAL_CURVE_H
#define RECIPROCAL_CURVE_H

#include "Plotter.h"
/**
 * 曲线类
 */
class ReciprocalCurve {
public:
  ReciprocalCurve(const double a = 0.0, const double b = 0.0,
                           const double c = 0.0, const double out_min = 0.0,
                           const double out_max = 1.0)
      : mA(a), mB(b), mC(c), mOutMin(out_min), mOutMax(out_max) {}

  const double A() const { return mA; }
  const double B() const { return mB; }
  const double C() const { return mC; }
  const double OutMin() const { return mOutMin; }
  const double OutMax() const { return mOutMax; }

  void SetABC(const double a, const double b, const double c) {
    mA = a;
    mB = b;
    mC = c;
  }

  void SetOutMinMax(const double out_min, const double out_max) {
    mOutMin = out_min;
    mOutMax = out_max;
  }

  void Interpolate(const double x1, const double y1, const double x2,
                   const double y2, const double x3, const double y3) {
    Assert(((x1 - x2) / (y2 - y1) - (x1 - x3) / (y3 - y1)) != 0);
    Assert((y1 - y2) != 0);

    mA = ((x1 * y1 - x2 * y2) / (y2 - y1) - (x1 * y1 - x3 * y3) / (y3 - y1)) /
         ((x1 - x2) / (y2 - y1) - (x1 - x3) / (y3 - y1));
    mC = (mA * (x1 - x2) - (x1 * y1 - x2 * y2)) / (y1 - y2);
    mB = (y1 - mA) * (x1 + mC);
    mB = (y2 - mA) * (x2 + mC);
    mB = (y3 - mA) * (x3 + mC);
  }

  double GetOutput(const double x, const bool limited = true) const {
    double value = mA + mB / (x + mC);
    if (limited == true) {
      if (value < mOutMin) {
        value = mOutMin;
      } else if (value > mOutMax) {
        value = mOutMax;
      }
    }
    return value;
  }

  void Show(const char *title, double minx, double maxx) {
    std::cerr << mA << " + " << mB << " / ( x + " << mC << " )" << std::endl;

    Plotter::instance().GnuplotExecute("set xrange [%g:%g]", minx, maxx);
    Plotter::instance().GnuplotExecute("plot %g + %g / (x + %g) title \"%s\"",
                                       mA, mB, mC, title);
  }

private:
  double mA;
  double mB;
  double mC;
  double mOutMin;
  double mOutMax;
};

#endif