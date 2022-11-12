#include "BehaviorAttack.h"
#include "BehaviorDribble.h"
#include "BehaviorHold.h"
#include "BehaviorIntercept.h"
#include "BehaviorPass.h"
#include "BehaviorPosition.h"
#include "BehaviorShoot.h"
#include "WorldState.h"

BehaviorAttackPlanner::BehaviorAttackPlanner(Agent &agent)
    : BehaviorPlannerBase<BehaviorAttackData>(agent) {}

BehaviorAttackPlanner::~BehaviorAttackPlanner() {}

void BehaviorAttackPlanner::Plan(std::list<ActiveBehavior> &behavior_list) 
{
  if (mSelfState.IsBallCatchable() && mStrategy.IsLastOppControl() &&
      (!(mAgent.IsLastActiveBehaviorInActOf(BT_Pass) ||
         mAgent.IsLastActiveBehaviorInActOf(BT_Dribble))))
    return;

  BehaviorInterceptPlanner(mAgent).Plan(mActiveBehaviorList);
  BehaviorShootPlanner(mAgent).Plan(mActiveBehaviorList);
  BehaviorPassPlanner(mAgent).Plan(mActiveBehaviorList);
  BehaviorDribblePlanner(mAgent).Plan(mActiveBehaviorList);
  BehaviorPositionPlanner(mAgent).Plan(mActiveBehaviorList);
  BehaviorHoldPlanner(mAgent).Plan(mActiveBehaviorList);

  if (!mActiveBehaviorList.empty()) {
    mActiveBehaviorList.sort(std::greater<ActiveBehavior>());
    behavior_list.push_back(mActiveBehaviorList.front());

    if (mActiveBehaviorList.size() > 1) { //允许非最优行为提交视觉请求
      double plus = 1.0;
      ActiveBehaviorPtr it = mActiveBehaviorList.begin();
      for (++it; it != mActiveBehaviorList.end(); ++it) {
        it->SubmitVisualRequest(plus);
        plus *= 2.0;
      }
    }
  }
}
