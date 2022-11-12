#ifndef __Coach_H__
#define __Coach_H__

#include "Client.h"
#include "Types.h"
#include "Utilities.h"

//using namespace std;
//class HeteroManager;

class Coach : public Client {
public:
  /**
   * 构造函数和析构函数
   */
  Coach() { };
  virtual ~Coach() { };

  void Run() override;
  void SendOptionToServer() override;
};

//struct PlayerCompare {
  //bool operator() (const std::pair<int, double> &i, const std::pair<int, double> &j) {
    //return i.second < j.second;
  //}
//};
#endif
