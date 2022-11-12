#include "Analyser.h"
#include "Agent.h"
#include "CommunicateSystem.h"
#include "Dasher.h"
#include "Formation.h"
#include "InterceptInfo.h"
#include "Logger.h"
#include "PositionInfo.h"
#include "VisualSystem.h"
#include <vector>
using namespace std;

Analyser::Analyser(Agent &agent) : DecisionData(agent) {}

Analyser::~Analyser() {}

void Analyser::UpdateRoutine() {
  mLightHouse = mAgent.GetStrategy().GetBallInterPos();

  for (Unum i = 1; i <= TEAMSIZE; ++i) {
    mHome[i] = mFormation.GetTeammateFormationPoint(i, mLightHouse);
  }
}
