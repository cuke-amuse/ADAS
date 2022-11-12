#ifndef BEHAVIORDEFENSE_H_
#define BEHAVIORDEFENSE_H_

#include <vector>

#include "Analyser.h"
#include "BehaviorBase.h"

class BehaviorDefensePlanner : public BehaviorPlannerBase<BehaviorDefenseData> {
public:
  BehaviorDefensePlanner(Agent &agent);
  virtual ~BehaviorDefensePlanner();

  void Plan(std::list<ActiveBehavior> &behavior_list) override;
};

#endif /* BEHAVIORDEFENSE_H_ */
