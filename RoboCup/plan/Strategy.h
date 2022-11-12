#ifndef __Strategy_H__
#define __Strategy_H__

#include <vector>

#include "BehaviorBase.h"
#include "DecisionData.h"
#include "Geometry.h"
#include "InterceptInfo.h"
#include "Types.h"

class Agent;
class PlayerState;

/**
 * 由于 Strategy 里得信息更新时，要用到自己得号码，所以不能放在 InfoState
 * 里面，现在移到 Agent 里
 */
class Strategy : public DecisionData {
public:
  explicit Strategy(Agent &agent);
  virtual ~Strategy();

  void UpdateRoutine();

private:
  void StrategyAnalyze();
  void BallPossessionAnalyse();
  bool SetPlayAnalyse();
  void SituationAnalyse();
  void StateAnalyse();

public:
  /** Situation based strategy position **/
  Vector GetTeammateSBSPPosition(Unum t_num, const Vector &ballpos);
  bool IsBallActuralKickable() const { return mIsBallActuralKickable; }
  bool IsBallFree() const { return mIsBallFree; }
  int GetBallFreeCycleLeft() const { return mBallFreeCycleLeft; }
  bool IsTmKickable() const;

  Unum GetController() const { return mController; }
  void SetController(Unum controller) { mController = controller; }
  bool IsMyControl() const;
  bool IsTmControl() const { return mController > 0; }
  bool IsOppControl() const { return mController < 0; }
  bool IsBallOutOfControl() const { return mController == 0; }

  bool IsLastMyControl() const;
  bool IsLastTmControl() const { return mLastController > 0; }
  bool IsLastOppControl() const { return mLastController < 0; }
  bool IsLastBallFree() const { return mIsLastBallFree; }
  Unum GetLastController() const { return mLastController; }
  Unum GetLastChallenger() const { return mLastChallenger; }

  const Time &GetLastBallFreeTime() const { return mLastBallFreeTime; }

  const Situation &GetSituation() const { return mSituation; }
  void SetSituation(Situation S) { mSituation = S; }

  const Vector &GetBallInterPos() const { return mBallInterPos; }

  int GetBallOutCycle() const { return mBallOutCycle; }

  int GetMyInterCycle() const { return mMyInterCycle; }
  void SetMyInterCycle(int cycle) { mMyInterCycle = cycle; }

  Vector GetMyInterPos();

  int GetMinOppInterCycle() const { return mMinOppInterCycle; }
  int GetMinTmInterCycle() const { return mMinTmInterCycle; }
  Unum GetFastestTm() const { return mFastestTm; }
  Unum GetFastestOpp() const { return mFastestOpp; }

  int GetSureOppInterCycle() const { return mSureOppInterCycle; }
  int GetSureTmInterCycle() const { return mSureTmInterCycle; }
  int GetSureInterCycle() const { return mSureInterCycle; }
  Unum GetSureTm() const { return mSureTm; }
  Unum GetSureOpp() const { return mSureOpp; }

private:
  bool mIsBallActuralKickable; //是否实际可踢
  bool mIsBallFree;
  int mBallFreeCycleLeft; // ball_free还能持续的周期

  Unum
      mController; //+表示自己人控球，-表示对方控球，0表示没人控球（有人控球跟球不是free不是一回事，球free是指当前无人可踢球，但控球的人不一定要能踢球，可以是最快截到球的人）
  Unum mChallenger; //对方可踢的人 +

  bool mIsLastBallFree;
  Time mLastBallFreeTime;
  Unum
      mLastController; //记录上次控球的人（可以是上个周期（比如在根结点），也可以是上个阶段的（比如在new的infostate里））
  Unum mLastChallenger;

  //下面是考虑cycle_delay的最小截球周期（就是认为cycle_delay时间内已经跑向球）
  int mMyInterCycle;
  int mMinTmInterCycle; /*这里的队友不包含自己*/
  int mMinOppInterCycle;
  int mMinIntercCycle;
  Unum mFastestTm;
  Unum mFastestOpp;

  //下面是综合考虑算出的最有可能的截球周期
  int mSureTmInterCycle;
  int mSureOppInterCycle;
  int mSureInterCycle;
  Unum mSureTm;
  Unum mSureOpp;

  Vector mBallInterPos; //球被最先截住的地方，可以是自己人和对方去截到
  int mBallOutCycle; //距离球出界的周期

  Situation mSituation; //当前的攻防形势

private:
  bool mForbiddenDribble; // setplay发球时不能带球

public:
  bool IsForbidenDribble() const { return mForbiddenDribble; }
  void SetForbidenDribble(const bool &forbiden) {
    mForbiddenDribble = forbiden;
  }

public:
  PlayerInterceptInfo
      mMyTackleInfo; //假设自己的可踢范围扩大到铲球距离的截球信息

  Vector AdjustTargetForSetplay(Vector target);

private:
  class VirtualSelf : public PlayerState {
  public:
    VirtualSelf(const PlayerState &player) : PlayerState(player) {}
    const double &GetKickableArea() const {
      return ServerParam::instance().maxTackleArea();
    }
  };

  /** 关于点球的接口和数据 */
public:
  void SetPenaltyFirstStep(bool flag) { mIsPenaltyFirstStep = flag; }
  bool IsPenaltyFirstStep() const { return mIsPenaltyFirstStep; }
  Unum GetPenaltyTaker() const { return mPenaltyTaker; }
  bool IsMyPenaltyTaken() const;

private:
  bool IsPenaltyPlayMode() const;
  void PenaltyAnalyze();

private:
  bool mIsPenaltyFirstStep; // 罚球者setup时用到的量
  int mPenaltySetupTime;    // 我方第几次罚点球
  Unum mPenaltyTaker;       // 罚点球的人，对手罚时为-1
};

#endif
