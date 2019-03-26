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

// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "cyber/cyber.h"

#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr size_t BUFFER_SIZE = 256;

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

// The NovAtel's orientation covariance matrix is pitch, roll, and yaw. We use
// the index array below
// to convert it to the orientation covariance matrix with order roll, pitch,
// and yaw.
constexpr int INDEX[] = {4, 3, 5, 1, 0, 2, 7, 6, 8};
static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

}  // namespace



class INS550DParser : public Parser {
public:
    INS550DParser();
    explicit INS550DParser(const config::Config& config);

    virtual MessageType GetMessage(MessagePtr *message_ptr);

private:
    bool HandleIns();
    bool HandleInsStat();

private:
    std::vector<uint8_t> buffer_;
    int messagetype;
    ::apollo::drivers::gnss::Ins ins_;
    ::apollo::drivers::gnss::InsStat ins_stat_;

    std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::Ins>>
      ins_writer_ = nullptr;
};

Parser* Parser::createINS550D(const config::Config& config) {
  return new INS550DParser(config);
}

INS550DParser::INS550DParser() {
  buffer_.reserve(BUFFER_SIZE);
  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

}

INS550DParser::INS550DParser(const config::Config& config) {
  buffer_.reserve(BUFFER_SIZE);
  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  messagetype = 0;

  node_ = apollo::cyber::CreateNode("INS550D_INS");
  ins_writer_ = node_->CreateWriter<apollo::drivers::gnss::Ins>("/apollo/sensor/gnss/ins");
}

Parser::MessageType INS550DParser::GetMessage(MessagePtr *message_ptr) {

  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  if(messagetype == 0)
  {
    messagetype = 1;

    if (HandleIns()==true) {
      *message_ptr = &ins_;
      ins_writer_->Write(ins_);
      return MessageType::INS;
    }

  }
  
  if(messagetype == 1)
  {
    messagetype = 2;

    if (HandleInsStat()==true) {
      *message_ptr = &ins_stat_;
      return MessageType::INS_STAT;
    }
  }

  messagetype = 0;
  return MessageType::NONE;
}

bool INS550DParser::HandleIns() {
int DataLen = 58;
uint8_t checksum = 0;
int16_t buf;
int32_t buf32;

if (data_[0] == 0xBD && data_[1] == 0xDB && data_[2] == 0x0B)
{
    for (int i = 0; i < DataLen - 1; i++)
    {
        checksum = checksum ^ data_[i];
    }


    if (data_[DataLen - 1] == checksum)
    {
        buf = (int16_t)(data_[3] + (data_[4] << 8));
        double roll = ((double)buf/32768) * 360;  // 横滚角

        buf = (int16_t)(data_[5] + (data_[6] << 8));
        double pitch = ((double)buf/32768) * 360; // 俯仰角

        buf = (int16_t)(data_[7] + (data_[8] << 8));
        double yaw = ((double)buf/32768) * 360;   // 方位角

        /*****************************************************/
        buf = (int16_t)(data_[9] + (data_[10] << 8));
        double gx = ((double)buf/32768) * 300;    // 陀螺 x 轴

        buf = (int16_t)(data_[11] + (data_[12] << 8));
        double gy = ((double)buf/32768) * 300;    // 陀螺 y 轴

        buf = (int16_t)(data_[13] + (data_[14] << 8));
        double gz = ((double)buf/32768) * 300;    // 陀螺 z 轴

        /*****************************************************/
        buf = (int16_t)(data_[15] + (data_[16] << 8));
        double ax = ((double)buf/32768) * 12;   // 加表 x 轴

        buf = (int16_t)(data_[17] + (data_[18] << 8));
        double ay = ((double)buf/32768) * 12;   // 加表 y 轴

        buf = (int16_t)(data_[19] + (data_[20] << 8));
        double az = ((double)buf/32768) * 12;   // 加表 z 轴

        /*****************************************************/
        buf32 = (int32_t)(data_[21] + (data_[22] << 8) + (data_[23] << 16) + (data_[24] << 24));
        double lat = ((double)buf32) / 1e7f;   // 纬度

        buf32 = (int32_t)(data_[25] + (data_[26] << 8) + (data_[27] << 16) + (data_[28] << 24));
        double lon = ((double)buf32) / 1e7f;   // 经度

        buf32 = (int32_t)(data_[29] + (data_[30] << 8) + (data_[31] << 16) + (data_[32] << 24));
        double alt = ((double)buf32) / 1e3f;    // 高度

        /*****************************************************/
        buf = (int16_t)(data_[33] + (data_[34] << 8));
        double north_velocity = ((double)buf/32768) * 100; // 北向速度
        buf = (int16_t)(data_[35] + (data_[36] << 8));
        double east_velocity = ((double)buf/32768) * 100; // 东向速度
        buf = (int16_t)(data_[37] + (data_[38] << 8));
        double up_velocity = ((double)buf/32768) * 100; // 地向速度


        uint8_t state = data_[39];    // 初校准状态

        /*****************************************************/

        int16_t data1 = (int16_t)(data_[46] + (data_[47] << 8));
        int16_t data2 = (int16_t)(data_[48] + (data_[49] << 8));
        int16_t data3 = (int16_t)(data_[50] + (data_[51] << 8));

        // buf = (uint32_t)(data_[46] + (data_[47] << 8));
        // double Temp = ((double)buf) * 200 / 32768;    // 温度

        buf = (uint32_t)(data_[52] + (data_[53] << 8) + (data_[54] << 16) + (data_[55] << 24));
        double gpstime = ((double)buf) / 4000;    // 时间

        uint8_t datatype = data_[56];   // 轮循数据类型

        /*****************************************************/
        ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());

        ins_.set_measurement_time(gpstime);   // 惯导发送的时间

        ins_.set_type(apollo::drivers::gnss::Ins::GOOD);

        ins_.mutable_position()->set_lon(lon);
        ins_.mutable_position()->set_lat(lat);
        ins_.mutable_position()->set_height(alt);

        ins_.mutable_euler_angles()->set_x(yaw);
        ins_.mutable_euler_angles()->set_y(pitch);
        ins_.mutable_euler_angles()->set_z(roll);
   
        ins_.mutable_linear_velocity()->set_x(east_velocity);
        ins_.mutable_linear_velocity()->set_y(north_velocity);
        ins_.mutable_linear_velocity()->set_z(up_velocity);

        ins_.mutable_angular_velocity()->set_x(gx);
        ins_.mutable_angular_velocity()->set_y(gy);
        ins_.mutable_angular_velocity()->set_z(gz);

        ins_.mutable_linear_acceleration()->set_x(ax);
        ins_.mutable_linear_acceleration()->set_y(ay);
        ins_.mutable_linear_acceleration()->set_z(az);

        ins_.set_state(state);              // 初校准状态

        if(datatype==32)                    // GPS 状态
        {
          ins_.set_positionstate(data1);    // GPS 定位状态
          ins_.set_numstars(data2);         // 收星数
          ins_.set_directionalstate(data3); // GPS 定向状态
        }

        return true;
        
        }
        
}
    return false;
}

bool INS550DParser::HandleInsStat() {
  ins_stat_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  ins_stat_.set_ins_status(3);
  ins_stat_.set_pos_type(56);
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo