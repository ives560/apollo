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
    std::vector<uint8_t> buffer_;

    ::apollo::drivers::gnss::Ins ins_;
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

}

Parser::MessageType INS550DParser::GetMessage(MessagePtr *message_ptr) {

int DataLen = 58;
uint8_t checksum = 0;
uint32_t buf;

if (buffer_[0] == 0xBD && buffer_[1] == 0xDB && buffer_[2] == 0x0B)
{
    for (int i = 0; i < DataLen - 1; i++)
    {
        checksum = checksum ^ buffer_[i];
    }

    AERROR << "GetMessage "<< "checksum=" <<checksum << "buffer_ = "<<buffer_[DataLen - 1];

    if (buffer_[DataLen - 1] == checksum)
    {
        buf = (uint32_t)(buffer_[3] + (buffer_[4] << 8));
        double roll = ((double)buf) * 360 / 32768;  // 横滚角

        buf = (uint32_t)(buffer_[5] + (buffer_[6] << 8));
        double pitch = ((double)buf) * 360 / 32768; // 俯仰角

        buf = (uint32_t)(buffer_[7] + (buffer_[8] << 8));
        double yaw = ((double)buf) * 360 / 32768;   // 方位角

        /*****************************************************/
        buf = (uint32_t)(buffer_[9] + (buffer_[10] << 8));
        double gx = ((double)buf) * 300 / 32768;    // 陀螺 x 轴

        buf = (uint32_t)(buffer_[11] + (buffer_[12] << 8));
        double gy = ((double)buf) * 300 / 32768;    // 陀螺 y 轴

        buf = (uint32_t)(buffer_[13] + (buffer_[14] << 8));
        double gz = ((double)buf) * 300 / 32768;    // 陀螺 z 轴

        /*****************************************************/
        buf = (uint32_t)(buffer_[15] + (buffer_[16] << 8));
        double ax = ((double)buf) * 12 / 32768;   // 加表 x 轴

        buf = (uint32_t)(buffer_[17] + (buffer_[18] << 8));
        double ay = ((double)buf) * 12 / 32768;   // 加表 y 轴

        buf = (uint32_t)(buffer_[19] + (buffer_[20] << 8));
        double az = ((double)buf) * 12 / 32768;   // 加表 z 轴

        /*****************************************************/
        buf = (uint32_t)(buffer_[21] + (buffer_[22] << 8) + (buffer_[23] << 16) + (buffer_[24] << 24));
        double lat = ((double)buf) / 1e7f;   // 纬度

        buf = (uint32_t)(buffer_[25] + (buffer_[26] << 8) + (buffer_[27] << 16) + (buffer_[28] << 24));
        double lon = ((double)buf) / 1e7f;   // 经度

        buf = (uint32_t)(buffer_[29] + (buffer_[30] << 8) + (buffer_[31] << 16) + (buffer_[32] << 24));
        double alt = ((double)buf) / 1e3f;    // 高度

        /*****************************************************/
        buf = (uint32_t)(buffer_[33] + (buffer_[34] << 8));
        double north_velocity = ((double)buf) * 100/ 32768; // 北向速度
        buf = (uint32_t)(buffer_[35] + (buffer_[36] << 8));
        double east_velocity = ((double)buf) * 100/ 32768; // 东向速度
        buf = (uint32_t)(buffer_[37] + (buffer_[38] << 8));
        double up_velocity = ((double)buf) * 100/ 32768; // 地向速度


        // uint8_t State = buffer_[39];    // 初校准状态

        // buf = (uint32_t)(buffer_[46] + (buffer_[47] << 8));
        // double Temp = ((double)buf) * 200 / 32768;    // 温度

        // buf = (uint32_t)(buffer_[52] + (buffer_[53] << 8) + (buffer_[54] << 16) + (buffer_[55] << 24));
        // double time = ((double)buf) / 4000;    // 时间

        // uint8_t datatype = buffer_[56];   // 轮循数据类型

        /*****************************************************/
        ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());

        ins_.set_type(apollo::drivers::gnss::Ins::GOOD);

        ins_.mutable_position()->set_lon(lon);
        ins_.mutable_position()->set_lat(lat);
        ins_.mutable_position()->set_height(alt);

        ins_.mutable_euler_angles()->set_x(roll);
        ins_.mutable_euler_angles()->set_y(pitch);
        ins_.mutable_euler_angles()->set_z(yaw);
   
        ins_.mutable_linear_velocity()->set_x(east_velocity);
        ins_.mutable_linear_velocity()->set_y(north_velocity);
        ins_.mutable_linear_velocity()->set_z(up_velocity);

        ins_.mutable_angular_velocity()->set_x(gx);
        ins_.mutable_angular_velocity()->set_y(gy);
        ins_.mutable_angular_velocity()->set_z(gz);

        ins_.mutable_linear_acceleration()->set_x(ax);
        ins_.mutable_linear_acceleration()->set_y(ay);
        ins_.mutable_linear_acceleration()->set_z(az);


        return MessageType::INS;
        
        }
        
}
    return MessageType::NONE;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo