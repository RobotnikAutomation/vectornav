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
#ifndef VECTORNAV__VECTORNAV_HPP
#define VECTORNAV__VECTORNAV_HPP

#include <vectornav/vectornav_params.hpp>
#include <vn/sensors.h>
#include <vn/compositedata.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vectornav/Ins.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <mutex>

namespace vectornav
{

struct Vectornav
{
  explicit Vectornav(ros::NodeHandle & nh, ros::NodeHandle & pnh);
  void spin();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  params params_{};
  bool has_rotation_reference_frame_ { false };

  vn::sensors::VnSensor vs_{};

  int device_family_ { 0 };

  unsigned int imu_stride_    { 0U };
  unsigned int output_stride_ { 0U };
  double newest_sensor_time_  { 0. };
  double biggest_ros_dt_      {-1. };
  double last_sensor_time_    { 0. };
  double maximum_imu_timestamp_difference_ { 0. };
  double average_time_difference_{ 0. };
  ros::Time ros_start_time_      { };
  ros::Time newest_timestamp_    { };
  bool adjust_ros_timestamp_     { false };
  int invalid_data_        { 0 };
  int max_invalid_packets_ { -1 };
  bool initial_position_set_  { false };
  vn::math::vec3d initial_position_ { };

  ros::Publisher pub_imu_, pub_mag_, pub_gps_, pub_odom_, pub_temp_, pub_pres_, pub_ins_;
  ros::ServiceServer srv_set_horizontal_, srv_reset_odom_, srv_reset_horizontal_;

  void read_parameters();
  void advertise_topics();
  void advertise_services();

  void connect_device(std::string const & port, int baudrate);
  void disconnect_device();
  void configure_device();

  std::mutex mtx_samples_;
  std::mutex service_acc_bias_mtx_;
  struct sample_t {double x, y, z;};
  bool take_samples_{false};
  std::vector<sample_t> samples_{};
  bool reset_horizontal(std_srvs::Trigger::Request const & req, std_srvs::Trigger::Response & res);
  bool set_horizontal(std_srvs::Trigger::Request const & req, std_srvs::Trigger::Response & res);
  bool reset_odometry(std_srvs::Empty::Request const & req, std_srvs::Empty::Response & res);

  static void binary_async_message_received(void* ,vn::protocol::uart::Packet & p, size_t index);
  void binary_async_message_received_(vn::protocol::uart::Packet & p, size_t index);
  std::uint64_t pkg_count_ { 0U };
  ros::Time get_timestamp(vn::sensors::CompositeData & cd, const ros::Time & ros_time);
  bool validate_timestamp(double const sensor_time);
  bool fill_imu_msg(vn::sensors::CompositeData & cd, sensor_msgs::Imu & imu_msg, const ros::Time & ros_time);
  bool fill_mag_msg(vn::sensors::CompositeData & cd, sensor_msgs::MagneticField & mag_msg, const ros::Time & ros_time);
  bool fill_temp_msg(vn::sensors::CompositeData & cd, sensor_msgs::Temperature & temp_msg, const ros::Time & ros_time);
  bool fill_pres_msg(vn::sensors::CompositeData & cd, sensor_msgs::FluidPressure & pres_msg, const ros::Time & ros_time);
  bool fill_gps_msg(vn::sensors::CompositeData & cd, sensor_msgs::NavSatFix & gps_msg, const ros::Time & ros_time);
  bool fill_odom_msg(vn::sensors::CompositeData & cd, nav_msgs::Odometry & odom_msg, const ros::Time & ros_time);
  bool fill_ins_msg(vn::sensors::CompositeData & cd, vectornav::Ins & ins_msg, const ros::Time & ros_time);
};

} // namespace vectornav

#endif  // VECTORNAV__VECTORNAV_HPP
