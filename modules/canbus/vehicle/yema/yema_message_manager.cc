/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/vehicle/yema/yema_message_manager.h"

#include "modules/canbus/vehicle/yema/protocol/brake_60.h"
#include "modules/canbus/vehicle/yema/protocol/steering_64.h"
#include "modules/canbus/vehicle/yema/protocol/throttle_62.h"
#include "modules/canbus/vehicle/yema/protocol/gear_66.h"
#include "modules/canbus/vehicle/yema/protocol/turnsignal_68.h"

namespace apollo {
namespace canbus {
namespace yema {

YeMaMessageManager::YeMaMessageManager() {
  // TODO(Authors): verify which one is recv/sent
  AddSendProtocolData<Brake60, true>();
  AddSendProtocolData<Throttle62, true>();
  AddSendProtocolData<Steering64, true>();
  AddSendProtocolData<Gear66, true>();
  AddSendProtocolData<Turnsignal68, true>();

}

YeMaMessageManager::~YeMaMessageManager() {}

}  // namespace yema
}  // namespace canbus
}  // namespace apollo
