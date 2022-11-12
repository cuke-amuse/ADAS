#ifndef DECISIONDATA_H_
#define DECISIONDATA_H_

#include "Utilities.h"

class Agent;
class WorldState;
class BallState;
class PlayerState;
class InfoState;
class Formation;

class DecisionData : public Updatable {
public:
    explicit DecisionData(Agent &agent);
    virtual ~DecisionData();
    void Update();
private:
    explicit DecisionData(const DecisionData &);
    const DecisionData &operator=(const DecisionData &);
protected:
    Agent &mAgent;
    const WorldState &mWorldState;
    const BallState &mBallState;
    const PlayerState &mSelfState;
    InfoState &mInfoState;
    Formation &mFormation;
};

#endif /* DECISIONDATA_H_ */
