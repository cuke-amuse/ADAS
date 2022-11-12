#ifndef __Player_H__
#define __Player_H__

#include <memory>
#include "Client.h"

class DecisionTree;
class BeliefState;

class Player : public Client {
public:
  /**
   * 构造函数和析构函数
   */
  Player();
  virtual ~Player();

  void Run() override;
  void SendOptionToServer() override;
private:
  std::shared_ptr<DecisionTree> mpDecisionTree{nullptr};
};

#endif
