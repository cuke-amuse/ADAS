#ifndef MATH_CALC_H
#define MATH_CALC_H 

#include <vector>

using namespace std;

class MathTools {
public:
static vector<double> linspace(double a, double b, int num, bool endpoint = true) {
  if (a > b)
    return linspace(b, a, num, endpoint);

  vector<double> res;
  double offset = endpoint ? (b - a) / (num - 1) : (b - a) / num;
  double val = a;
  while (val < b) {
    res.push_back(val);
    val += offset;
  }
  return res;
}

static double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

static double norm(vector<double> a) {
  double sum = 0;
  for (int i = 0; i < a.size(); i++) {
    sum += a[i] * a[i];
  }
  return sqrt(sum);
}

static double cross(vector<double> &A, vector<double> &B) {
  return A[0] * B[1] - A[1] * B[0];
}
};
#endif