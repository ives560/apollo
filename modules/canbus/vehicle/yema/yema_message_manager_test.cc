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

#include "gtest/gtest.h"

#include "modules/canbus/vehicle/yema/protocol/brake_60.h"
#include "modules/canbus/vehicle/yema/protocol/steering_64.h"
#include "modules/canbus/vehicle/yema/protocol/throttle_62.h"
#include "modules/canbus/vehicle/yema/protocol/turnsignal_68.h"

namespace apollo {
namespace canbus {
namespace yema {

using ::apollo::drivers::canbus::ProtocolData;
using ::apollo::canbus::ChassisDetail;

class YeMaMessageManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(YeMaMessageManagerTest, Brake60) {
  YeMaMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Brake60::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Brake60 *>(pd)->ID, Brake60::ID);
}

TEST_F(YeMaMessageManagerTest, Steering64) {
  YeMaMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Steering64::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Steering64 *>(pd)->ID, Steering64::ID);
}

TEST_F(YeMaMessageManagerTest, Throttle62) {
  YeMaMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Throttle62::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Throttle62 *>(pd)->ID, Throttle62::ID);
}

}  // namespace yema
}  // namespace canbus
}  // namespace apollo
