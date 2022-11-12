#include "InfoState.h"
#include "InterceptInfo.h"
#include "PositionInfo.h"
#include <memory>

InfoState::InfoState(WorldState *world_state) {
  mpPositionInfo = std::make_shared<PositionInfo>(world_state, this);
  mpInterceptInfo = std::make_shared<InterceptInfo>(world_state, this);
}

InfoState::~InfoState() {
  //delete mpPositionInfo;
  //delete mpInterceptInfo;
}

PositionInfo &InfoState::GetPositionInfo() const {
  mpPositionInfo->Update();
  return *mpPositionInfo;
}

InterceptInfo &InfoState::GetInterceptInfo() const {
  mpInterceptInfo->Update();
  return *mpInterceptInfo;
}
