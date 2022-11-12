#ifndef MOBILE_STATE_H
#define MOBILE_STATE_H

#include "BaseState.h"

class MobileState : public BaseState {
public:
  MobileState(double decay, double effective_speed_max)
      : mDecay(decay), mEffectiveSpeedMax(effective_speed_max),
        mpPredictor(new Predictor(mDecay)), mGuessed(0) {
    mVelEps = 10000;
    ResetPredictor();
  }

 // class Cls; cls a; cls b = a or cls b(a)
  MobileState(const MobileState &m)
      : BaseState(m), mDecay(m.GetDecay()),
        mEffectiveSpeedMax(m.GetEffectiveSpeedMax()),
        mpPredictor(new Predictor(mDecay)), mGuessed(0) {
    mVel = m.mVel;
    mVelEps = m.mVelEps;
    UpdatePos(m.GetPos(), m.GetPosDelay(), m.GetPosConf());
  }

  virtual ~MobileState() { delete mpPredictor; }

  void UpdatePos(const Vector &pos, int delay = 0, double conf = 1.0);

  // Class Cls; Cls a; Cls b; b = a;
  MobileState &operator=(const MobileState &other) {   // 
    mVel = other.mVel;
    mDecay = other.mDecay;
    UpdatePos(other.GetPos(), other.GetPosDelay(), other.GetPosConf()); // use old predictor
    mEffectiveSpeedMax = other.mEffectiveSpeedMax;

    return *this;  //
  }

  /**更新速度
   * @param 速度
   * @param 时间
   * @param 置信度*/
  void UpdateVel(const Vector &vel, int delay = 0, double conf = 1);

  /**获得当前或前几次的速度
   * @param 最新值之前的周期数，范围为0到RECORDSIZE-1,否则会循环
   */
  const Vector &GetVel() const { return mVel.mValue; }

  /**获得当前或前几次的速度的时间
   * @param 最新值之前的周期数，范围为0到RECORDSIZE-1,否则会循环
   */
  int GetVelDelay() const { return mVel.mCycleDelay; }

  /**获得当前或前几次的速度的置信度
   * @param 最新值之前的周期数，范围为0到RECORDSIZE-1,否则会循环
   */
  const double &GetVelConf() const { return mVel.mConf; }

  /**
   * 计算速度误差
   */
  void UpdateVelEps(double eps) { mVelEps = eps; }

  /**
   * 获得速度误差
   */
  double GetVelEps() const { return mVelEps; };

  /**
   * 得到运动 decay
   * @return
   */
  const double &GetDecay() const { return mDecay; }

  void SetDecay(const double &decay) { mDecay = decay; }

  /**
   * 实际最大速度
   */
  const double &GetEffectiveSpeedMax() const { return mEffectiveSpeedMax; }
  void SetEffectiveSpeedMax(double effective_speed_max) {
    mEffectiveSpeedMax = effective_speed_max;
  }

  /**内部自更新
   * @param 每周期delay加上的数据
   * @param 每周期conf衰减的数据*/
  void AutoUpdate(int delay_add = 1, double conf_dec_factor = 1);

public:
  class Predictor {
  public:
    Predictor(const double &decay) : mDecay(decay) {}

    enum { MAX_STEP = 50 };

    const Vector &GetPredictedPos(int step = 1) {
      step = Min(step, int(MAX_STEP));

      if (step <= mPredictedStep) {
        return mPredictedPos[step];
      } else {
        PredictTo(step);

        return mPredictedPos[step];
      }
    }

    const Vector &GetPredictedVel(int step = 1) {
      step = Min(step, int(MAX_STEP));

      if (step <= mPredictedStep) {
        return mPredictedVel[step];
      } else {
        PredictTo(step);

        return mPredictedVel[step];
      }
    }

    void UpdatePosAndVel(const Vector &pos, const Vector &vel) {
      mPredictedPos[0] = pos;
      mPredictedVel[0] = vel;
      mPredictedStep = 0;
    }

  private:
    void PredictTo(const int &step) {
      for (int i = mPredictedStep + 1; i <= step; ++i) {
        mPredictedPos[i] = mPredictedPos[i - 1] + mPredictedVel[i - 1];
        mPredictedVel[i] = mPredictedVel[i - 1] * mDecay;
      }
      mPredictedStep = step;
    }

    const double mDecay;

    Array<Vector, MAX_STEP + 1> mPredictedPos;
    Array<Vector, MAX_STEP + 1> mPredictedVel;
    int mPredictedStep;
  };

  void ResetPredictor() { mpPredictor->UpdatePosAndVel(GetPos(), GetVel()); }

public:
  const Vector &GetPredictedPos(int step = 1) const {
    return mpPredictor->GetPredictedPos(step);
  }

  const Vector &GetPredictedVel(int step = 1) const {
    return mpPredictor->GetPredictedVel(step);
  }

  Vector GetFinalPos() const {
    return GetPos() + GetVel() / (1.0 - GetDecay());
  }

public:
  // guessed time is now only for WorldStateUpdater , other should not use it;
  /** set guessed times*/
  void UpdateGuessedTimes(int guessed) {
    Assert(guessed >= 0);
    mGuessed = guessed;
  }

  /** get guessed times*/
  int GetGuessedTimes() const { return mGuessed; }

private:
  /**存储数个周期的速度*/
  StateValue<Vector> mVel;

  double mVelEps;

  double mDecay;
  double mEffectiveSpeedMax;

private:
  Predictor *mpPredictor;

  int mGuessed; // indicate that the times of this object ot be guessed -- only
                // used from WorldStateUpdater::Run
};

#endif
