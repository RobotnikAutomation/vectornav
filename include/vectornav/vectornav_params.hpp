// Copyright (c) 2022, Robotnik Automation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once
#ifndef VECTORNAV__VECTORNAV_PARAMS_HPP
#define VECTORNAV__VECTORNAV_PARAMS_HPP

#include <string>
#include <vector>

namespace vectornav
{

struct params
{
  std::string map_frame_id { "map" };
  std::string imu_frame_id { "imu" };

  std::string port { "/dev/ttyUSB0" };
  int baudrate     { 921600 };

  bool tf_ned_to_enu         { false };
  bool frame_based_enu       { false };

  bool tf_ned_to_nwu         { false };
  bool frame_based_nwu       { false };

  bool adjust_ros_timestamp  { false };
  int async_output_rate      { 200 };
  int imu_output_rate        { 200 };
  int fixed_imu_rate         { 800 };
  int max_invalid_packets    { 500 };

  bool acc_bias_enable       { false };
  double set_acc_bias_seconds{ 2.5 };

  std::vector<double> linear_accel_covariance{0.01, 0.00, 0.00,
                                              0.00, 0.01, 0.00,
                                              0.00, 0.00, 0.01};

  std::vector<double> angular_vel_covariance{0.01, 0.00, 0.00,
                                             0.00, 0.01, 0.00,
                                             0.00, 0.00, 0.01};

  std::vector<double> orientation_covariance{0.01, 0.00, 0.00,
                                             0.00, 0.01, 0.00,
                                             0.00, 0.00, 0.01};

  std::vector<double> rotation_reference_frame{1., 0., 0.,
                                               0., 1., 0.,
                                               0., 0., 1.};
};

} // namespace vectornav

#endif // VECTORNAV__VECTORNAV_PARAMS_HPP