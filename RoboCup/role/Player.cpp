#include "Player.h"

#include "Agent.h"
#include "CommandSender.h"
#include "CommunicateSystem.h"
#include "Dasher.h"
#include "DecisionTree.h"
#include "DynamicDebug.h"
#include "Formation.h"
#include "Logger.h"
#include "Parser.h"
#include "Thread.h"
#include "TimeTest.h"
#include "UDPSocket.h"
#include "VisualSystem.h"
#include "WorldModel.h"
#include <memory>

Player::Player() { 
 mpDecisionTree = std::make_shared<DecisionTree>();
}

Player::~Player() { //delete mpDecisionTree;
 }

void Player::SendOptionToServer() {
  while (!mpParser->IsClangOk()) {
    mpAgent->CheckCommands(mpObserver);
    mpAgent->Clang(7, 8);
    mpObserver->SetCommandSend();
    WaitFor(200);
  }

  while (!mpParser->IsSyncOk()) {
    mpAgent->CheckCommands(mpObserver);
    mpAgent->SynchSee();
    mpObserver->SetCommandSend();
    WaitFor(200);
  }

  mpAgent->CheckCommands(mpObserver);
  mpAgent->EarOff(false);
  mpObserver->SetCommandSend();
  WaitFor(200);
}

void Player::Run() {
  // TIMETEST("Run");

  static Time last_time = Time(-100, 0);

  mpObserver->Lock();

  /** 下面几个更新顺序不能变 */
  Formation::instance.SetTeammateFormations();
  CommunicateSystem::instance().Update();   //在这里解析hear信息，必须首先更新
  mpAgent->CheckCommands(mpObserver);
  mpWorldModel->Update(mpObserver);

  mpObserver->UnLock();

  const Time &time = mpAgent->GetWorldState().CurrentTime();

  if (last_time.T() >= 0) {
    if (time != Time(last_time.T() + 1, 0) &&
        time != Time(last_time.T(), last_time.S() + 1)) {
      if (time == last_time) {
        mpAgent->World().SetCurrentTime(
            Time(last_time.T(), last_time.S() + 1)); //否则决策数据更新会出问题
      }
    }
  }

  last_time = time;

  Formation::instance.UpdateOpponentRole(); // TODO:
                             // 暂时放在这里，教练未发来对手阵型信息时自己先计算

  VisualSystem::instance().ResetVisualRequest();
  mpDecisionTree->Decision(*mpAgent);

  VisualSystem::instance().Decision();
  CommunicateSystem::instance().Decision();

  if (ServerParam::instance().synchMode()) {
    mpAgent->Done();
  }

  mpAgent->SetHistoryActiveBehaviors();

  Logger::instance().LogSight();
}
