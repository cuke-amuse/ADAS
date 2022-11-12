#ifndef ANALYSER_H_
#define ANALYSER_H_

#include <vector>

#include "DecisionData.h"
#include "Geometry.h"
#include "InterceptInfo.h"
#include "ServerParam.h"
#include "Utilities.h"

class Agent;

/**
 * 主要是防守需要使用的一些数据
 * */
class Analyser : public DecisionData {
public:
  explicit Analyser(Agent &agent);
  virtual ~Analyser();

  void UpdateRoutine();
  /**
   * broadcast the position of ball & opponents
   */
  void BroadcastPosition();
//private:
  Vector mLightHouse;
  PlayerArray<Vector, true> mHome;
};

#endif /* ANALYSER_H_ */
