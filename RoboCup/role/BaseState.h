#ifndef __BASESTATE_H__
#define __BASESTATE_H__

#include <cstring>
#include "PlayerParam.h"
#include "ServerParam.h"
//#include "MobileState.h"

//最基本的值
template <class T> 
class StateValue {
public:
  StateValue() : mValue(T()), mCycleDelay(9999), mConf(0) {}

  explicit StateValue(const StateValue &o) {
    mValue = o.mValue;
    mCycleDelay = o.mCycleDelay;
    mConf = o.mConf;
  }

  /**内部自更新
   * @param 每周期delay加上的数据
   * @param 每周期conf衰减的数据*/
  void AutoUpdate(int delay_add = 1, double conf_dec_factor = 1) {
    mCycleDelay += delay_add;
    mConf *= conf_dec_factor;
  }

public:
  /**值大小*/
  T mValue;

  /**值距当前时间*/
  int mCycleDelay;

  /**值的可信度*/
  double mConf;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/**最基础的BaseState类*/
class BaseState {
public:
  BaseState() {
    mPos.mValue = Vector(10000, 10000);
    mPosEps = 10000;
  }

  explicit BaseState(const BaseState &o) { mPos = o.mPos; }

  /**更新位置
   * @param 位置
   * @param 时间
   * @param 置信度*/
  void UpdatePos(const Vector &pos, int delay = 0, double conf = 1);

  /**内部自更新
   * @param 每周期delay加上的数据
   * @param 每周期conf衰减的数据*/
  void AutoUpdate(int delay_add = 1, double conf_dec_factor = 1);
    /**
   *  设置pos的误差范围,主要指误差圆的半径
   */
  void UpdatePosEps(double eps) { mPosEps = eps; }

  /**获得当前或前几次的位置
   * @param 最新值之前的周期数，范围为0到RECORDSIZE-1,否则会循环
   */
  const Vector &GetPos() const { return mPos.mValue; }

  /**获得当前或前几次的时间
   * @param 最新值之前的周期数，范围为0到RECORDSIZE-1,否则会循环
   */
  int GetPosDelay() const { return mPos.mCycleDelay; }

  /**获得当前或前几次的置信度
   * @param 最新值之前的周期数，范围为0到RECORDSIZE-1,否则会循环
   */
  const double GetPosConf() const { return mPos.mConf; }

    /**
   * 获得pos的误差的eps
   */
  const double GetPosEps() const { return mPosEps; }
private:
  /**存储数周期的位置*/
  StateValue<Vector> mPos;

  double mPosEps;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
/**静态物体基类*/
/**注：暂时没想到有什么信息*/
class StaticState : public BaseState {};


#endif
