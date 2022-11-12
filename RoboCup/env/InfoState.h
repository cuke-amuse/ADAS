/************************************************************************************
 * WrightEagle (Soccer Simulation League 2D) * BASE SOURCE CODE RELEASE 2016 *
 * Copyright (c) 1998-2016 WrightEagle 2D Soccer Simulation Team, * Multi-Agent
 *Systems Lab.,                                * School of Computer Science and
 *Technology,               * University of Science and Technology of China *
 * All rights reserved. *
 *                                                                                  *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the WrightEagle 2D Soccer Simulation Team nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                  *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED    * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *PURPOSE ARE           * DISCLAIMED. IN NO EVENT SHALL WrightEagle 2D Soccer
 *Simulation Team BE LIABLE    * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *EXEMPLARY, OR CONSEQUENTIAL       * DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *PROCUREMENT OF SUBSTITUTE GOODS OR       * SERVICES; LOSS OF USE, DATA, OR
 *PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       * CAUSED AND ON ANY THEORY OF
 *LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,    * OR TORT (INCLUDING
 *NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF * THIS SOFTWARE,
 *EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                *
 ************************************************************************************/

#ifndef INFOSTATE_H_
#define INFOSTATE_H_

#include <memory>

#include "WorldState.h"

class InfoState;
class InterceptInfo;
class PositionInfo;

/**
 * 这些是由 WorldState 计算出来得信息状态
 */
class InfoStateBase : public Updatable {
public:
  InfoStateBase(const WorldState *world_state, const InfoState *info_state)
      : mpWorldState(world_state), mpInfoState(info_state) {}
  virtual ~InfoStateBase() {}

  void Update() { UpdateAtTime(mpWorldState->CurrentTime()); }

protected:
  const WorldState *mpWorldState;
  const InfoState *mpInfoState;
};

class InfoState {
  friend class Agent;
  InfoState(InfoState &);

public:
  InfoState(WorldState *world_state);
  virtual ~InfoState();

  PositionInfo& GetPositionInfo() const;
  InterceptInfo &GetInterceptInfo() const;

private:
  std::shared_ptr<PositionInfo> mpPositionInfo{nullptr};
  std::shared_ptr<InterceptInfo> mpInterceptInfo{nullptr};
};

#endif /* INFOSTATE_H_ */
