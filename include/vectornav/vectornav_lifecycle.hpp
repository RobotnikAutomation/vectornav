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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vectornav/msg/ins.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mutex>

namespace vectornav
{

struct VectornavLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
  explicit VectornavLifecycleNode();
  ~VectornavLifecycleNode();

private:
  params params_{};
  
  std::shared_ptr<vn::sensors::VnSensor> vs_{ nullptr };

  int device_family_ { 0 };

  unsigned int imu_stride_    { 0U };
  unsigned int output_stride_ { 0U };
  double newest_sensor_time_  { 0. };
  double biggest_ros_dt_      {-1. };
  double last_sensor_time_    { 0. };
  double maximum_imu_timestamp_difference_ { 0. };
  double average_time_difference_{ 0. };
  rclcpp::Time ros_start_time_   { 0 };
  rclcpp::Time newest_timestamp_ { 0 };
  int invalid_data_        { 0 };
  int max_invalid_packets_ { -1 };
  bool initial_position_set_  { false };
  vn::math::vec3d initial_position_ { };

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher
    <sensor_msgs::msg::Imu>::SharedPtr pub_imu_{ nullptr };
  rclcpp_lifecycle::LifecyclePublisher
    <sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_{ nullptr };
  rclcpp_lifecycle::LifecyclePublisher
    <sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_{ nullptr };
  rclcpp_lifecycle::LifecyclePublisher
    <nav_msgs::msg::Odometry>::SharedPtr pub_odom_{ nullptr };
  rclcpp_lifecycle::LifecyclePublisher
    <sensor_msgs::msg::Temperature>::SharedPtr pub_temp_{ nullptr };
  rclcpp_lifecycle::LifecyclePublisher
    <sensor_msgs::msg::FluidPressure>::SharedPtr pub_pres_{ nullptr };
  rclcpp_lifecycle::LifecyclePublisher
    <vectornav::msg::Ins>::SharedPtr pub_ins_{ nullptr };

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    srv_set_acc_bias_{ nullptr }, srv_reset_acc_bias_{ nullptr };
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    srv_reset_device_{ nullptr }, srv_tare_device_{ nullptr };
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr
    srv_reset_odom_{ nullptr };

  void read_parameters();
  void advertise_topics();
  void advertise_services();
  void delete_topics();
  void delete_services();

  void connect_device(std::string const & port, int baudrate);
  void disconnect_device();
  void configure_device();
  void unconfigure_device();

  std::mutex mtx_samples_;
  std::mutex service_acc_bias_mtx_;
  struct sample_t {double x, y, z;};
  bool take_samples_{false};
  std::vector<sample_t> samples_{}; 
  void set_acc_bias(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void reset_acc_bias(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void reset_device(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void tare_device(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void reset_odometry(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> resp);
  
  static void binary_async_message_received(void* ,vn::protocol::uart::Packet & p, size_t index);
  void binary_async_message_received_(vn::protocol::uart::Packet & p, size_t index);
  std::uint64_t pkg_count_ { 0U };
  rclcpp::Time get_timestamp(vn::sensors::CompositeData & cd, const rclcpp::Time & ros_time);
  bool validate_timestamp(double const sensor_time);
  bool fill_imu_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::Imu & imu_msg, const rclcpp::Time & ros_time);
  bool fill_mag_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::MagneticField & mag_msg, const rclcpp::Time & ros_time);
  bool fill_temp_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::Temperature & temp_msg, const rclcpp::Time & ros_time);
  bool fill_pres_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::FluidPressure & pres_msg, const rclcpp::Time & ros_time);
  bool fill_gps_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::NavSatFix & gps_msg, const rclcpp::Time & ros_time);
  bool fill_odom_msg(vn::sensors::CompositeData & cd, nav_msgs::msg::Odometry & odom_msg, const rclcpp::Time & ros_time);
  bool fill_ins_msg(vn::sensors::CompositeData & cd, vectornav::msg::Ins & ins_msg, const rclcpp::Time & ros_time);

private: // rclcpp_lifecycle
  using cb_return = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  cb_return on_configure(const rclcpp_lifecycle::State & state);
  cb_return on_activate(const rclcpp_lifecycle::State & state);
  cb_return on_deactivate(const rclcpp_lifecycle::State & state);
  cb_return on_cleanup(const rclcpp_lifecycle::State & state);
  cb_return on_shutdown(const rclcpp_lifecycle::State & state);
  cb_return on_error(const rclcpp_lifecycle::State & state);

  rclcpp::TimerBase::SharedPtr main_loop_timer_;
  void main_loop();
};

} // namespace vectornav

#endif  // VECTORNAV__VECTORNAV_HPP
