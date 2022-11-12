#include "BehaviorShoot.h"

#include <cmath>
#include <list>

#include "ActionEffector.h"
#include "BehaviorBase.h"
#include "Dasher.h"
#include "Evaluation.h"
#include "Geometry.h"
#include "InterceptModel.h"
#include "Kicker.h"
#include "PlayerParam.h"
#include "PositionInfo.h"
#include "ServerParam.h"
#include "Tackler.h"
#include "TimeTest.h"
#include "Utilities.h"
#include "VisualSystem.h"

using namespace std;

namespace {
bool ret = BehaviorExecutable::AutoRegister<BehaviorShootExecuter>();
}

const BehaviorType BehaviorShootExecuter::BEHAVIOR_TYPE = BT_Shoot;

/**
 * BehaviorShootExecuter's constructor
 */
BehaviorShootExecuter::BehaviorShootExecuter(Agent &agent)
    : BehaviorExecuterBase<BehaviorAttackData>(agent) {
  Assert(ret);
}

/**
 * BehaviorShootExecuter's constructor
 */
BehaviorShootExecuter::~BehaviorShootExecuter() { }

/**
 * Execute an active behavior as shoot. May raise ball or player.
 * \param act_bhv
 * \return true  if success;
 *         false otherwise.
 */
bool BehaviorShootExecuter::Execute(const ActiveBehavior &shoot) {
  Logger::instance().LogShoot(mBallState.GetPos(), shoot.mTarget, "@Shoot");
  if (shoot.mDetailType == BDT_Shoot_Tackle) { 
    return Tackler::instance().TackleToDir(mAgent, shoot.mAngle);
  } else { 
    return Kicker::instance().KickBall(mAgent, shoot.mTarget,
                                       ServerParam::instance().ballSpeedMax(),
                                       KM_Quick, 0, true);
  }
}

/**
 * BehaviorShootPlanner's constructor
 */
BehaviorShootPlanner::BehaviorShootPlanner(Agent &agent)
    : BehaviorPlannerBase<BehaviorAttackData>(agent) {}

/**
 * BehaviorShootPlanner's destructor
 */
BehaviorShootPlanner::~BehaviorShootPlanner() {}

/**
 * Plan.
 * None or one ActiveBehavior will be push back to behavior_list.
 */
void BehaviorShootPlanner::Plan(list<ActiveBehavior> &behavior_list) 
{
  if (!mSelfState.IsKickable())
    return;

  if (mWorldState.GetPlayMode() == PM_Our_Foul_Charge_Kick ||
      mWorldState.GetPlayMode() == PM_Our_Back_Pass_Kick ||
      mWorldState.GetPlayMode() == PM_Our_Indirect_Free_Kick ||
      (mWorldState.GetLastPlayMode() == PM_Our_Indirect_Free_Kick &&
       mAgent.IsLastActiveBehaviorInActOf(
           BT_Pass))) // Indircet后传球保持不射门，至少跑位后上个动作会改
  {
    return;
  }

  if (mSelfState.GetPos().X() >
      ServerParam::instance().pitchRectanglar().Right() -
          PlayerParam::instance().shootMaxDistance()) {
    AngleDeg left =
        (ServerParam::instance().oppLeftGoalPost() - mSelfState.GetPos()).Dir();
    AngleDeg right =
        (ServerParam::instance().oppRightGoalPost() - mSelfState.GetPos()).Dir();
    Vector target;
    AngleDeg interval;
    Line c(ServerParam::instance().oppLeftGoalPost(),
           ServerParam::instance().oppRightGoalPost());
    AngleDeg shootDir =
        mPositionInfo.GetShootAngle(left, right, mSelfState, interval);
    if (interval < mSelfState.GetRandAngle(
                       ServerParam::instance().maxPower(),
                       ServerParam::instance().ballSpeedMax(), mBallState) * 3) {
      return;
    }

    Ray f(mSelfState.GetPos(), shootDir);
    c.Intersection(f, target);
    if (Tackler::instance().CanTackleToDir(mAgent, shootDir) &&
        Tackler::instance().GetBallVelAfterTackle(mAgent, shootDir).Mod() >
            ServerParam::instance().ballSpeedMax() - 0.05) {
      ActiveBehavior shoot(mAgent, BT_Shoot, BDT_Shoot_Tackle);
      shoot.mTarget = target;
      shoot.mAngle = shootDir;
      shoot.mEvaluation = 2.0 + FLOAT_EPS;
      behavior_list.push_back(shoot);
    } else {
      ActiveBehavior shoot(mAgent, BT_Shoot);
      shoot.mTarget = target;
      shoot.mEvaluation = 2.0 + FLOAT_EPS;

      behavior_list.push_back(shoot);
    }
  }
}
