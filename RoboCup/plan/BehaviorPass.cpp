#include "BehaviorPass.h"

#include <sstream>

#include "Agent.h"
#include "CommunicateSystem.h"
#include "Dasher.h"
#include "Evaluation.h"
#include "Formation.h"
#include "InfoState.h"
#include "Kicker.h"
#include "Logger.h"
#include "PositionInfo.h"
#include "Strategy.h"
#include "Tackler.h"
#include "TimeTest.h"
#include "Types.h"
#include "VisualSystem.h"
#include "WorldState.h"

using namespace std;

const BehaviorType BehaviorPassExecuter::BEHAVIOR_TYPE = BT_Pass;

namespace {
bool ret = BehaviorExecutable::AutoRegister<BehaviorPassExecuter>();
}

BehaviorPassExecuter::BehaviorPassExecuter(Agent &agent)
    : BehaviorExecuterBase<BehaviorAttackData>(agent) {
  Assert(ret);
}

BehaviorPassExecuter::~BehaviorPassExecuter(void) {}

bool BehaviorPassExecuter::Execute(const ActiveBehavior &pass) {
  Logger::instance().LogPass(false, mBallState.GetPos(), pass.mTarget, "@Pass",
                             true);
  if (pass.mDetailType == BDT_Pass_Direct)
    return Kicker::instance().KickBall(mAgent, pass.mTarget, pass.mKickSpeed,
                                       KM_Quick);
  if (pass.mDetailType == BDT_Pass_Clear) {
    //		Logger::instance().GetTextLogger("Clear")<<
    //mWorldState.CurrentTime().S() + "Clear";
    if (Tackler::instance().CanTackleToDir(mAgent, pass.mAngle) &&
        Tackler::instance().GetBallVelAfterTackle(mAgent, pass.mAngle).Mod() >
            Kicker::instance().GetMaxSpeed(mAgent, pass.mTarget, 1) - 0.08)
      return Tackler::instance().TackleToDir(mAgent, pass.mAngle);
    else
      return Kicker::instance().KickBall(mAgent, pass.mTarget,
                                         ServerParam::instance().ballSpeedMax(),
                                         KM_Quick, 0);
  }
  return false;
}
BehaviorPassPlanner::BehaviorPassPlanner(Agent &agent)
    : BehaviorPlannerBase<BehaviorAttackData>(agent) {}

BehaviorPassPlanner::~BehaviorPassPlanner(void) {}

void BehaviorPassPlanner::Plan(std::list<ActiveBehavior> &behavior_list) {
  if (!mSelfState.IsKickable())
    return;

  const std::vector<Unum> &tm2ball =
      mPositionInfo.GetCloseTeammateToTeammate(mSelfState.GetUnum());

  Unum _opp = mStrategy.GetFastestOpp();
  if (!_opp)
    return;

  PlayerState oppState = mWorldState.GetOpponent(_opp);
  bool oppClose = oppState.IsKickable() || oppState.GetTackleProb(true) > 0.65;
  for (uint i = 0; i < tm2ball.size(); ++i) {
    ActiveBehavior pass(mAgent, BT_Pass);

    pass.mTarget = mWorldState.GetTeammate(tm2ball[i]).GetPredictedPos();

    if (mWorldState.GetTeammate(tm2ball[i]).IsGoalie()) {
      continue;
    }
    Vector rel_target = pass.mTarget - mBallState.GetPos();
    const std::vector<Unum> &opp2tm =
        mPositionInfo.GetCloseOpponentToTeammate(tm2ball[i]);
    AngleDeg min_differ = HUGE_VALUE;

    for (uint j = 0; j < opp2tm.size(); ++j) {
      Vector rel_pos =
          mWorldState.GetOpponent(opp2tm[j]).GetPos() - mBallState.GetPos();
      if (rel_pos.Mod() > rel_target.Mod() + 3.0)
        continue;

      AngleDeg differ = GetAngleDegDiffer(rel_target.Dir(), rel_pos.Dir());
      if (differ < min_differ) {
        min_differ = differ;
      }
    }

    if (min_differ < 10.0)
      continue;

    pass.mEvaluation =
        Evaluation::instance().EvaluatePosition(pass.mTarget, true);

    pass.mAngle = (pass.mTarget - mSelfState.GetPos()).Dir();
    pass.mKickSpeed = ServerParam::instance().GetBallSpeed(
        5, pass.mTarget.Dist(mBallState.GetPos()));
    pass.mKickSpeed =
        MinMax(2.0, pass.mKickSpeed,
               Kicker::instance().GetMaxSpeed(mAgent, pass.mAngle, 3));
    if (oppClose) { // in oppnent control, clear it
      pass.mDetailType = BDT_Pass_Clear;
    } else
      pass.mDetailType = BDT_Pass_Direct;
    mActiveBehaviorList.push_back(pass);
  }
  if (!mActiveBehaviorList.empty()) {
    mActiveBehaviorList.sort(std::greater<ActiveBehavior>());
    if (mActiveBehaviorList.front().mDetailType == BDT_Pass_Clear) {
      mActiveBehaviorList.front().mEvaluation = 1.0 + FLOAT_EPS;
    }
    behavior_list.push_back(mActiveBehaviorList.front());
  } else { //如果此周期没有好的动作
    if (mAgent.IsLastActiveBehaviorInActOf(BT_Pass)) {
      ActiveBehavior pass(mAgent, BT_Pass, BDT_Pass_Direct);
      pass.mTarget = mAgent.GetLastActiveBehaviorInAct()->mTarget; //行为保持
      pass.mEvaluation =
          Evaluation::instance().EvaluatePosition(pass.mTarget, true);
      pass.mKickSpeed = ServerParam::instance().GetBallSpeed(
          5 + random() % 6, pass.mTarget.Dist(mBallState.GetPos()));
      pass.mKickSpeed =
          MinMax(2.0, pass.mKickSpeed, ServerParam::instance().ballSpeedMax());
      behavior_list.push_back(pass);
    }
    if (oppClose) {
      Vector p;
      BallState SimBall = mBallState;
      int MinTmInter = HUGE_VALUE, MinOppInter = HUGE_VALUE, MinTm;
      Vector MinTmPos;
      for (AngleDeg dir = -45; dir <= 45; dir += 2.5) {
        if (!Tackler::instance().CanTackleToDir(mAgent, dir)) {
          SimBall.UpdateVel(
              Polar2Vector(Kicker::instance().GetMaxSpeed(
                               mAgent, mSelfState.GetBodyDir() + dir, 1),
                           mSelfState.GetBodyDir() + dir),
              0, 1.0);
        } else
          SimBall.UpdateVel(
              Polar2Vector(Max(Tackler::instance()
                                   .GetBallVelAfterTackle(mAgent, dir)
                                   .Mod(),
                               Kicker::instance().GetMaxSpeed(
                                   mAgent, mSelfState.GetBodyDir() + dir, 1)),
                           mSelfState.GetBodyDir() + dir),
              0, 1.0);
        for (int i = 2; i <= 11; i++) {
          if (fabs((mWorldState.GetTeammate(i).GetPos() - mSelfState.GetPos())
                       .Dir() -
                   dir) > 45) {
            continue;
          }
          if (!mWorldState.GetPlayer(i).IsAlive()) {
            continue;
          }
          PlayerInterceptInfo *a = mInterceptInfo.GetPlayerInterceptInfo(i);
          mInterceptInfo.CalcTightInterception(SimBall, a, true);
          if (MinTmInter > (*a).mMinCycle) {
            MinTm = i;
            MinTmInter = (*a).mMinCycle;
            MinTmPos = (*a).mInterPos;
          }
        }
        for (int i = 1; i <= 11; i++) {
          if (fabs((mWorldState.GetOpponent(i).GetPos() - mSelfState.GetPos())
                       .Dir() -
                   dir) > 45) {
            continue;
          }
          PlayerInterceptInfo *a = mInterceptInfo.GetPlayerInterceptInfo(-i);
          mInterceptInfo.CalcTightInterception(SimBall, a, true);
          if (MinOppInter > (*a).mMinCycle) {
            MinOppInter = (*a).mMinCycle;
          }
        }
        if (MinOppInter > MinTmInter) {
          Ray a(mSelfState.GetPos(), mSelfState.GetBodyDir() + dir);
          Line c(ServerParam::instance().ourLeftGoalPost(),
                 ServerParam::instance().ourRightGoalPost());
          c.Intersection(a, p);
          if (p.Y() < ServerParam::instance().ourPenaltyArea().Top() ||
              p.Y() > ServerParam::instance().ourPenaltyArea().Bottom()) {

            ActiveBehavior pass(mAgent, BT_Pass, BDT_Pass_Clear);
            pass.mTarget = MinTmPos;
            pass.mEvaluation = 1.0 + FLOAT_EPS;
            pass.mAngle = mSelfState.GetBodyDir() + dir;
            mActiveBehaviorList.push_back(pass);
          }
        }
      }
      if (!mActiveBehaviorList.empty()) {
        mActiveBehaviorList.sort(std::greater<ActiveBehavior>());
        behavior_list.push_back(mActiveBehaviorList.front());
      }
    }
  }
}
