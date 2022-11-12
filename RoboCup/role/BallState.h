#ifndef __BALLSTATE_H__
#define __BALLSTATE_H__

#include "MobileState.h"

class BallState : public MobileState {
public:
  BallState()
      : MobileState(ServerParam::instance().ballDecay(),
                    ServerParam::instance().ballSpeedMax()) {}
  virtual ~BallState() = default;
  void GetReverseFrom(const BallState &o) {
    UpdatePos(o.GetPos().Rotate(180.0), o.GetPosDelay(), o.GetPosConf());
    UpdateVel(o.GetVel().Rotate(180.0), o.GetVelDelay(), o.GetVelConf());
  }

private:
  /** 暂时有何种其它信息待定*/
};

#endif
