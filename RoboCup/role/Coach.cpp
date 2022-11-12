#include "Coach.h"

#include <iostream>
#include <vector>
#include <list>
#include <sstream>
#include <string>
#include <algorithm>

#include "Agent.h"
#include "CommandSender.h"
#include "DynamicDebug.h"
#include "Formation.h"
#include "Logger.h"
#include "Parser.h"
#include "PlayerParam.h"
#include "Thread.h"
#include "TimeTest.h"
#include "UDPSocket.h"
#include "WorldModel.h"

void Coach::SendOptionToServer() 
{
  while (!mpParser->IsEyeOnOk()) {
    UDPSocket::instance().Send("(eye on)");
    WaitFor(200);
  }
  std::vector<std::pair<int, double>> a;
  a.reserve(17);
  for (int i = 0; i <= 17; ++i) { //这里的18不知道如何去引用PlayerParam::DEFAULT_PLAYER_TYPES
    a.push_back(std::pair<int, double>(
        i, PlayerParam::instance().HeteroPlayer(i).effectiveSpeedMax()));
  }
  std::sort(a.begin(), a.end(), [] (const std::pair<int, double> &i, const std::pair<int, double> &j) {
    return i.second < j.second;
  });
  
  for (int i = 1; i <= TEAMSIZE; ++i) {
    if (i != PlayerParam::instance().ourGoalieUnum() &&
        mpObserver->Teammate_Fullstate(i).IsAlive()) {
      mpAgent->CheckCommands(mpObserver);
      if (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType ==
          LT_Forward) {
        mpAgent->ChangePlayerType(i, a.back().first);
        a.pop_back();
      }
      mpObserver->SetCommandSend();
      WaitFor(5);
    }
  }
  for (int i = 1; i <= TEAMSIZE; ++i) {
    if (i != PlayerParam::instance().ourGoalieUnum() &&
        mpObserver->Teammate_Fullstate(i).IsAlive()) {
      mpAgent->CheckCommands(mpObserver);
      if (mpAgent->GetFormation().GetTeammateRoleType(i).mLineType ==
          LT_Defender) {
        mpAgent->ChangePlayerType(i, a.back().first);
        a.pop_back();
      }
      mpObserver->SetCommandSend();
      WaitFor(5);
    }
  }
  if (mpObserver->Teammate_Fullstate(PlayerParam::instance().ourGoalieUnum())
          .IsAlive()) {
    double maxAcceleration = 0;
    std::vector<std::pair<int, double>>::iterator goalie;
    for (std::vector<std::pair<int, double>>::iterator it = a.begin(); it != a.end();
         ++it) {
      if (PlayerParam::instance().HeteroPlayer((*it).first).dashPowerRate() >
          maxAcceleration) {
        maxAcceleration =
            PlayerParam::instance().HeteroPlayer((*it).first).dashPowerRate();
        goalie = it;
      }
    }
    mpAgent->CheckCommands(mpObserver);
    mpAgent->ChangePlayerType(PlayerParam::instance().ourGoalieUnum(),
                              (*goalie).first);
    a.erase(goalie);
  }

  mpObserver->SetCommandSend();
  for (int i = 1; i <= TEAMSIZE; ++i) {
    if (i != PlayerParam::instance().ourGoalieUnum() &&
        mpObserver->Teammate_Fullstate(i).IsAlive()) {
      mpAgent->CheckCommands(mpObserver);
      if (mpAgent->GetFormation().GetPlayerRoleType(i).mLineType ==
          LT_Midfielder) {
        mpAgent->ChangePlayerType(i, a.back().first);
        a.pop_back();
      }
      mpObserver->SetCommandSend();
      WaitFor(5);
    }
  }

  /** check whether each player's type is changed OK */
  WaitFor(200);
  /*	for (int i = 1; i <= TEAMSIZE; ++i)
      {
          if (i != PlayerParam::instance().ourGoalieUnum() &&
     mpObserver->Teammate_Fullstate(i).IsAlive())
          {
              while (!mpParser->IsChangePlayerTypeOk(i) &&
     mpObserver->CurrentTime().T() < 1)
              {
                              mpAgent->CheckCommands(mpObserver);
                              mpAgent->ChangePlayerType(i, i);
                              mpObserver->SetCommandSend();
                  WaitFor(200);
                      }
          }
          }
          */
  /*	a.clear();
          for (int i=1 ;i <= 18 ; ++i){
     //这里的18不知道如何去引用PlayerParam::DEFAULT_PLAYER_TYPES
                  a.push_back(pair<int ,
     double>(i,PlayerParam::instance().HeteroPlayer(i).kickRand()));
          }
          for (int i = 1; i <= TEAMSIZE; ++i)
      {
          if (i != PlayerParam::instance().ourGoalieUnum() &&
     mpObserver->Teammate_Fullstate(i).IsAlive())
          {
                  mpAgent->CheckCommands(mpObserver);
                  if(mpAgent->GetFormation().GetPlayerRoleType(i).mLineType==LT_Forward)
                          mpAgent->ChangePlayerType(i, a.back().first);
              mpObserver->SetCommandSend();
              WaitFor(5);
          }
          }
          for (int i = 1; i <= TEAMSIZE; ++i)
      {
          if (i != PlayerParam::instance().ourGoalieUnum() &&
     mpObserver->Teammate_Fullstate(i).IsAlive())
          {
                  mpAgent->CheckCommands(mpObserver);
                  if(mpAgent->GetFormation().GetPlayerRoleType(i).mLineType==LT_Defender)
                          mpAgent->ChangePlayerType(i, a.back().first);
              mpObserver->SetCommandSend();
              WaitFor(5);
          }
          }
          for (int i = 1; i <= TEAMSIZE; ++i)
      {
          if (i != PlayerParam::instance().ourGoalieUnum() &&
     mpObserver->Teammate_Fullstate(i).IsAlive())
          {
                  mpAgent->CheckCommands(mpObserver);
                  if(mpAgent->GetFormation().GetPlayerRoleType(i).mLineType==LT_Midfielder)
                          mpAgent->ChangePlayerType(i, a.back().first);
              mpObserver->SetCommandSend();
              WaitFor(5);
          }
          }*/
}

/*
 * The main loop for coach.
 */
void Coach::Run() {
  mpObserver->Lock();

  /** 下面几个更新顺序不能变 */
  Formation::instance.SetTeammateFormations();
  mpAgent->CheckCommands(mpObserver); // 检查上周期发送命令情况
  mpWorldModel->Update(mpObserver);

  mpObserver->UnLock();

  // do planning...

  if (ServerParam::instance().synchMode()) {
    mpAgent->Done();
  }

  Logger::instance().LogSight();
}
