#include "BehaviorBlock.h"
#include "Agent.h"
#include "BasicCommand.h"
#include "BehaviorPosition.h"
#include "Dasher.h"
#include "Evaluation.h"
#include "Formation.h"
#include "Logger.h"
#include "PositionInfo.h"
#include "VisualSystem.h"

const BehaviorType BehaviorBlockExecuter::BEHAVIOR_TYPE = BT_Block;

namespace {
bool ret = BehaviorExecutable::AutoRegister<BehaviorBlockExecuter>();
}

BehaviorBlockExecuter::BehaviorBlockExecuter(Agent &agent)
    : BehaviorExecuterBase<BehaviorDefenseData>(agent) {
  Assert(ret);
}

BehaviorBlockExecuter::~BehaviorBlockExecuter(void) {}

bool BehaviorBlockExecuter::Execute(const ActiveBehavior &beh) {
  Logger::instance().LogGoToPoint(mSelfState.GetPos(), beh.mTarget, "@Block");

  return Dasher::instance().GoToPoint(mAgent, beh.mTarget, beh.mBuffer,
                                      beh.mPower, true, false);
}

BehaviorBlockPlanner::BehaviorBlockPlanner(Agent &agent)
    : BehaviorPlannerBase<BehaviorDefenseData>(agent) {}

BehaviorBlockPlanner::~BehaviorBlockPlanner() {}

void BehaviorBlockPlanner::Plan(std::list<ActiveBehavior> &behavior_list) {
  Unum closest_tm = mPositionInfo.GetClosestTeammateToBall();
  if (mWorldState.GetPlayMode() >= PM_Opp_Corner_Kick &&
      mWorldState.GetPlayMode() <= PM_Opp_Offside_Kick) {
    return;
  }

  if (closest_tm == mSelfState.GetUnum() ||
      mStrategy.GetMyInterCycle() <= mStrategy.GetSureTmInterCycle()) {
    ActiveBehavior block(mAgent, BT_Block);

    block.mBuffer = 0.5;
    block.mPower = mSelfState.CorrectDashPowerForStamina(
        ServerParam::instance().maxDashPower());
    block.mTarget = mAnalyser.mLightHouse;
    block.mEvaluation = 1.0 + FLOAT_EPS;

    behavior_list.push_back(block);
  }
}
