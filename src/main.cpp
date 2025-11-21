/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <cmath>
#include <iostream>

// No need to define PI twice if we already have it included...
//#define M_PI 3.14159265358979323846  /* M_PI */

// ROS Libraries
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vectornav/msg/ins.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/rclcpp.hpp>

#include <mutex>

rclcpp::Node::SharedPtr node = nullptr;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubIMU;
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pubMag;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGPS;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom;
rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pubTemp;
rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pubPres;
rclcpp::Publisher<vectornav::msg::Ins>::SharedPtr pubIns;

rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetOdomSrv;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr setHorizontalSrv, resetHorizontalSrv, resetDeviceSrv, tareDeviceSrv;

//XmlRpc::XmlRpcValue rpc_temp;

std::mutex mtx_samples;
struct sample_t{double x, y, z;};
bool take_samples{false};
std::vector<sample_t> samples{};

// Include this header file to get access to VectorNav sensors.
#include "vn/compositedata.h"
#include "vn/sensors.h"
#include "vn/util.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index);
bool ValidateQuaternion(vec4f q);
bool ValidateVector(vec3f v);
int invalid_data = 0;
// Number of consecutive invalid data packets allowed before shutting node down.
int max_invalid_packets = -1;
// Save newest timestamp to avoid publishing older messages
rclcpp::Time newest_timestamp;

// Custom user data to pass to packet callback function
struct UserData
{
  // the vectornav device identifier
  int device_family;
  // frame id used only for Odom header.frame_id
  std::string map_frame_id;
  // frame id used for header.frame_id of other messages and for Odom child_frame_id
  std::string frame_id;
  // Select ROS publication reference frame.
  // tf_ned_to_enu and tf_ned_to_nwu can not be used simultaneously.
  // Both to false to use NED (imu default frame), tf_ned_to_enu to use ENU, tf_ned_to_nwu to use NWU (ROS default frame)
  // Use frame_based_xxx along with tf_ned_to_xxx if the imu tf frame matches the IMU default frame (not tested yet)
  bool tf_ned_to_enu;
  bool frame_based_enu;
  bool tf_ned_to_nwu;
  bool frame_based_nwu;
  // Initial position after getting a GPS fix.
  vec3d initial_position;
  bool initial_position_set = false;
  bool acc_bias_enable;
  double set_acc_bias_seconds;

  //Unused covariances initialized to zero's
  std::vector<double> linear_accel_covariance = {};
  std::vector<double> angular_vel_covariance = {};
  std::vector<double> orientation_covariance = {};
  // Default rotation reference frame, to set different mounting positions
  std::vector<double> rotation_reference_frame = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  // ROS header time stamp adjustments
  double average_time_difference{0};
  rclcpp::Time ros_start_time;
  bool adjust_ros_timestamp{false};

  // sensor_time and ros_dt should always increase
  // Store values to discard unexpected measurements
  double newest_sensor_time{0};
  double biggest_ros_dt{-1.0};
  double last_sensor_time{0};
  double maximum_imu_timestamp_difference{};

  // strides
  unsigned int imu_stride;
  unsigned int output_stride;
};

bool validateSensorTimestamp(const double sensor_time, UserData * user_data);

// Reset initial position to current position
void resetOdom(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> resp, UserData * user_data)
{
  RCLCPP_INFO(node->get_logger(), "Reset Odometry");
  user_data->initial_position_set = false;
}

std::mutex service_acc_bias_mtx;

void reset_horizontal(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp, VnSensor* vs_ptr)
{
  std::unique_lock<std::mutex> service_lock(service_acc_bias_mtx);
  RCLCPP_INFO(node->get_logger(), "Reset to factory new acceleration bias");

  vn::math::mat3f const gain {1., 0., 0.,
                              0., 1., 0.,
                              0., 0., 1.};
  vs_ptr->writeAccelerationCompensation(gain, {0., 0., 0.}, true);
  vs_ptr->writeSettings(true);

  resp->success = true;
  resp->message = "Bias register set to zero. Please, reset vectornav hardware to avoid angular velocity.";
  RCLCPP_INFO(node->get_logger(), "Done.");
}

void reset_device(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp, VnSensor* vs_ptr)
{
  std::unique_lock<std::mutex> service_lock(service_acc_bias_mtx);
  RCLCPP_INFO(node->get_logger(), "Reset vectornav device");

  vs_ptr->reset();

  resp->success = true;
  resp->message = "Device reset command sent.";
  RCLCPP_INFO(node->get_logger(), "Done.");
}

void tare_device(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp, VnSensor* vs_ptr)
{
  std::unique_lock<std::mutex> service_lock(service_acc_bias_mtx);
  RCLCPP_INFO(node->get_logger(), "Tare vectornav device (zero angular velocity and acceleration)");

  vs_ptr->tare();

  resp->success = true;
  resp->message = "Device tare command sent.";
  RCLCPP_INFO(node->get_logger(), "Done.");
}

// Calibrate bias of accelerometer.
void set_horizontal(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp, VnSensor* vs_ptr, int *SensorImuRate, double* set_acc_bias_seconds)
{
  std::unique_lock<std::mutex> sample_lock(mtx_samples, std::defer_lock);
  std::unique_lock<std::mutex> service_lock(service_acc_bias_mtx);
  RCLCPP_INFO(node->get_logger(), "Set acceleration bias to zero (0.0, 0.0, 9.81)");
  vn::math::mat3f const gain {1., 0., 0.,
                              0., 1., 0.,
                              0., 0., 1.};

  sample_lock.lock();
    samples.clear();
    samples.reserve(static_cast<size_t>((*set_acc_bias_seconds) * (*SensorImuRate) * 1.5));
    take_samples = true;
    auto const start = node->get_clock()->now();
  sample_lock.unlock();

  node->get_clock()->sleep_for(rclcpp::Duration{std::chrono::microseconds{static_cast<uint64_t>(1e6*(*set_acc_bias_seconds))}});

  sample_lock.lock();
    auto const end = node->get_clock()->now();
    take_samples = false;

    // Calculate mean of samples
    double bias_x{0.}, bias_y{0.}, bias_z{0.};
    for (auto const & sample : samples) {
      bias_x += sample.x; bias_y += sample.y; bias_z += sample.z;
    }
    bias_x /= samples.size(); bias_y /= samples.size(); bias_z /= samples.size();

    // Calculate covariance of samples
    double covariance_x{0.}, covariance_y{0.}, covariance_z{0.};
    for (auto const & sample : samples) {
      covariance_x += (sample.x - bias_x) * (sample.x - bias_x);
      covariance_y += (sample.y - bias_y) * (sample.y - bias_y);
      covariance_z += (sample.z - bias_z) * (sample.z - bias_z);
    }
    covariance_x /= samples.size(); covariance_y /= samples.size(); covariance_z /= samples.size();
  sample_lock.unlock();

  auto const curr { vs_ptr->readAccelerationCompensation() };

  vn::math::vec3f const bias {curr.b.x+static_cast<float>(bias_x),
                              curr.b.y-static_cast<float>(bias_y),
                              curr.b.z-static_cast<float>(bias_z-9.80665)};

  if (samples.size() < 10) {
    RCLCPP_ERROR(node->get_logger(), "Not enough samples taken. Aborting.");
    RCLCPP_ERROR(node->get_logger(), "Not enough samples taken (<10). Aborting.");
    resp->message = "Not enough samples taken (<10). Aborting.";
    resp->success = false;
  } else {
    RCLCPP_INFO(node->get_logger(), "Applying bias correction to vectornav:");
    RCLCPP_INFO(node->get_logger(), " - Samples taked: %lu (%.2lfs)", samples.size(), (end - start).seconds());
    RCLCPP_INFO(node->get_logger(), " - Bias:       [x: %7.4lf, y: %7.4lf, z: %7.4lf]", bias.x, bias.y, bias.z);
    RCLCPP_INFO(node->get_logger(), " - Covariance: [x: %7.4lf, y: %7.4lf, z: %7.4lf]", covariance_x, covariance_y, covariance_z);
    resp->message = "Applying bias correction to vectornav, see log for more info. Please, reset vectornav hardware to avoid angular velocity.";
    resp->success = true;
    vs_ptr->writeAccelerationCompensation(gain, {bias.x, bias.y, bias.z}, true);
    vs_ptr->writeSettings(true);
    RCLCPP_INFO(node->get_logger(), "Done.");
  }
}

// Assure that the serial port is set to async low latency in order to reduce delays and package pilup.
// These changes will stay effective until the device is unplugged
#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
bool optimize_serial_communication(std::string portName)
{
  int portFd = -1;

  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    RCLCPP_WARN(node->get_logger(),  "Can't open port for optimization");
    return false;
  }

  RCLCPP_INFO(node->get_logger(),  "Set port to ASYNCY_LOW_LATENCY");
  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ::close(portFd);
  return true;
}
#elif
bool optimize_serial_communication(str::string portName) { return true; }
#endif

int main(int argc, char * argv[])
{
  // keeping all information passed to callback
  UserData user_data;

  // ROS node init
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("imu");

  pubIMU = node->create_publisher<sensor_msgs::msg::Imu>("~/data", 1000);
  pubMag = node->create_publisher<sensor_msgs::msg::MagneticField>("~/mag", 1000);
  pubGPS = node->create_publisher<sensor_msgs::msg::NavSatFix>("~/global_position/raw/fix", 1000);
  pubOdom = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", 1000);
  pubTemp = node->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 1000);
  pubPres = node->create_publisher<sensor_msgs::msg::FluidPressure>("~/atm_pressure", 1000);
  pubIns = node->create_publisher<vectornav::msg::Ins>("~/INS", 1000);

  resetOdomSrv = node->create_service<std_srvs::srv::Empty>(
    "~/reset_odom", std::bind(&resetOdom, std::placeholders::_1, std::placeholders::_2, &user_data));

  // Serial Port Settings
  string SensorPort;
  int SensorBaudrate;
  int async_output_rate;
  int imu_output_rate;

  // Sensor IMURATE (800Hz by default, used to configure device)
  int SensorImuRate;

  // Indicates whether a rotation reference frame has been read
  bool has_rotation_reference_frame = false;

  newest_timestamp = node->get_clock()->now();

  // Load all params
  node->declare_parameter<std::string>("map_frame_id", "map");
  node->declare_parameter<std::string>("frame_id", "vectornav");
  node->declare_parameter<bool>("tf_ned_to_enu", false);
  node->declare_parameter<bool>("frame_based_enu", false);
  node->declare_parameter<bool>("tf_ned_to_nwu", true);
  node->declare_parameter<bool>("frame_based_nwu", false);
  node->declare_parameter<bool>("adjust_ros_timestamp", false);
  node->declare_parameter<int>("async_output_rate", 40);
  node->declare_parameter<int>("imu_output_rate", 40);
  node->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
  node->declare_parameter<int>("serial_baud", 921600);
  node->declare_parameter<int>("fixed_imu_rate", 800);
  node->declare_parameter<int>("max_invalid_packets", 500);
  node->declare_parameter<bool>("acc_bias_enable", false);
  node->declare_parameter<double>("set_acc_bias_seconds", 2.5);

  node->declare_parameter<std::vector<double>>("linear_accel_covariance", {0.0003, 0, 0, 0, 0.0003, 0, 0, 0, 0.0003});
  node->declare_parameter<std::vector<double>>("angular_vel_covariance", {0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02});
  node->declare_parameter<std::vector<double>>("orientation_covariance", {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01});
  node->declare_parameter<bool>("has_rotation_reference_frame", false);
  node->declare_parameter<std::vector<double>>("rotation_reference_frame", {1., 0, 0, 0, 1., 0, 0, 0, 1.});

  node->get_parameter<std::string>("map_frame_id", user_data.map_frame_id);
  node->get_parameter<std::string>("frame_id", user_data.frame_id);
  node->get_parameter<bool>("tf_ned_to_enu", user_data.tf_ned_to_enu);
  node->get_parameter<bool>("frame_based_enu", user_data.frame_based_enu);
  node->get_parameter<bool>("tf_ned_to_nwu", user_data.tf_ned_to_nwu);
  node->get_parameter<bool>("frame_based_nwu", user_data.frame_based_nwu);
  node->get_parameter<bool>("adjust_ros_timestamp", user_data.adjust_ros_timestamp);
  node->get_parameter<int>("async_output_rate", async_output_rate);
  node->get_parameter<int>("imu_output_rate", imu_output_rate);
  node->get_parameter<std::string>("serial_port", SensorPort);
  node->get_parameter<int>("serial_baud", SensorBaudrate);
  node->get_parameter<int>("fixed_imu_rate", SensorImuRate);
  node->get_parameter<int>("max_invalid_packets", max_invalid_packets);
  node->get_parameter<bool>("acc_bias_enable", user_data.acc_bias_enable);
  node->get_parameter<double>("set_acc_bias_seconds", user_data.set_acc_bias_seconds);

  node->get_parameter("linear_accel_covariance", user_data.linear_accel_covariance);
  node->get_parameter("angular_vel_covariance", user_data.angular_vel_covariance);
  node->get_parameter("orientation_covariance", user_data.orientation_covariance);
  node->get_parameter<bool>("has_rotation_reference_frame", has_rotation_reference_frame);
  node->get_parameter("rotation_reference_frame", user_data.rotation_reference_frame);

  if (user_data.tf_ned_to_enu && user_data.tf_ned_to_nwu)
  {
    RCLCPP_ERROR(node->get_logger(),  "Unable to set both tf_ned_to_enu and tf_ned_to_nwu to true simultaneously. "
    "Please use just one of them to get data in the ENO or NWU frame, or set both "
    "to false to get data in the NED frame");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node->get_logger(),  "Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

  // try to optimize the serial port
  optimize_serial_communication(SensorPort);

  // Create a VnSensor object and connect to sensor
  VnSensor vs;

  // Default baudrate variable
  int defaultBaudrate;
  // Run through all of the acceptable baud rates until we are connected
  // Looping in case someone has changed the default
  bool baudSet = false;
  // Lets add the set baudrate to the top of the list, so that it will try
  // to connect with that value first (speed initialization up)
  std::vector<unsigned int> supportedBaudrates = vs.supportedBaudrates();
  supportedBaudrates.insert(supportedBaudrates.begin(), SensorBaudrate);
  while (!baudSet) {
    // Make this variable only accessible in the while loop
    static int i = 0;
    defaultBaudrate = supportedBaudrates[i];
    RCLCPP_INFO(node->get_logger(),  "Connecting with default at %d", defaultBaudrate);
    // Default response was too low and retransmit time was too long by default.
    // They would cause errors
    vs.setResponseTimeoutMs(1000);  // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);    // Retransmit every 50 ms

    // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
    // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
    // All other values seem to work fine.
    try {
      // Connect to sensor at it's default rate
      if (defaultBaudrate != 128000 && SensorBaudrate != 128000) {
        vs.connect(SensorPort, defaultBaudrate);
        // Issues a change baudrate to the VectorNav sensor and then
        // reconnects the attached serial port at the new baudrate.
        vs.changeBaudRate(SensorBaudrate);
        // Only makes it here once we have the default correct
        RCLCPP_INFO(node->get_logger(),  "Connected baud rate is %d", vs.baudrate());
        baudSet = true;
      }
    }
    // Catch all oddities
    catch (...) {
      // Disconnect if we had the wrong default and we were connected
      vs.disconnect();
      node->get_clock()->sleep_for(rclcpp::Duration{std::chrono::microseconds{static_cast<uint64_t>(1e6*0.2)}});
    }
    // Increment the default iterator
    i++;
    // There are only 9 available data rates, if no connection
    // made yet possibly a hardware malfunction?
    if (i > 8) {
      break;
    }
  }

  // Now we verify connection (Should be good if we made it this far)
  if (vs.verifySensorConnectivity()) {
    RCLCPP_INFO(node->get_logger(),  "Device connection established");
  } else {
    RCLCPP_ERROR(node->get_logger(),  "No device communication");
    RCLCPP_WARN(node->get_logger(),  "Please input a valid baud rate. Valid are:");
    RCLCPP_WARN(node->get_logger(),  "9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
    RCLCPP_WARN(node->get_logger(),  "With the test IMU 128000 did not work, all others worked fine.");
  }
  // Query the sensor's model number.
  string mn = vs.readModelNumber();
  string fv = vs.readFirmwareVersion();
  uint32_t hv = vs.readHardwareRevision();
  uint32_t sn = vs.readSerialNumber();
  RCLCPP_INFO(node->get_logger(),  "Model Number: %s, Firmware Version: %s", mn.c_str(), fv.c_str());
  RCLCPP_INFO(node->get_logger(),  "Hardware Revision : %d, Serial Number : %d", hv, sn);

  // calculate the least common multiple of the two rate and assure it is a
  // valid package rate, also calculate the imu and output strides
  int package_rate = 0;
  for (int allowed_rate : {1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200, 0}) {
    package_rate = allowed_rate;
    if ((package_rate % async_output_rate) == 0 && (package_rate % imu_output_rate) == 0) break;
  }
  if (package_rate == 0) {
    RCLCPP_ERROR(node->get_logger(),  "imu_output_rate (%d) or async_output_rate (%d) is not in 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, "
    "200 Hz",
    imu_output_rate, async_output_rate);
  }
  user_data.imu_stride = package_rate / imu_output_rate;
  user_data.output_stride = package_rate / async_output_rate;
  RCLCPP_INFO(node->get_logger(),  "Package Receive Rate: %d Hz", package_rate);
  RCLCPP_INFO(node->get_logger(),  "General Publish Rate: %d Hz", async_output_rate);
  RCLCPP_INFO(node->get_logger(),  "IMU Publish Rate: %d Hz", imu_output_rate);

  // Arbitrary value that indicates the maximum expected time between consecutive IMU messages
  user_data.maximum_imu_timestamp_difference = (1 / static_cast<double>(imu_output_rate)) * 10;

  // Set the device info for passing to the packet callback function
  user_data.device_family = vs.determineDeviceFamily();

  // Make sure no generic async output is registered
  vs.writeAsyncDataOutputType(VNOFF);

  // Configure binary output message
  BinaryOutputRegister bor(
    ASYNCMODE_PORT1,
    SensorImuRate / package_rate,  // update rate [ms]
    COMMONGROUP_QUATERNION | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE |
      COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES |
      (user_data.adjust_ros_timestamp ? COMMONGROUP_TIMESTARTUP : 0),
    TIMEGROUP_NONE | TIMEGROUP_GPSTOW | TIMEGROUP_GPSWEEK | TIMEGROUP_TIMEUTC, IMUGROUP_NONE,
    GPSGROUP_NONE,
    ATTITUDEGROUP_YPRU,  //<-- returning yaw pitch roll uncertainties
    INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELBODY | INSGROUP_ACCELECEF |
      INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU,
    GPSGROUP_NONE);

  // An empty output register for disabling output 2 and 3 if previously set
  BinaryOutputRegister bor_none(
    0, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
    INSGROUP_NONE, GPSGROUP_NONE);

  vs.writeBinaryOutput1(bor);
  vs.writeBinaryOutput2(bor_none);
  vs.writeBinaryOutput3(bor_none);

  // Setting reference frame
  vn::math::mat3f current_rotation_reference_frame;
  current_rotation_reference_frame = vs.readReferenceFrameRotation();
  RCLCPP_INFO_STREAM(node->get_logger(), "Current rotation reference frame: " << current_rotation_reference_frame);

  if (has_rotation_reference_frame == true) {
    vn::math::mat3f matrix_rotation_reference_frame;
    matrix_rotation_reference_frame.e00 = user_data.rotation_reference_frame[0];
    matrix_rotation_reference_frame.e01 = user_data.rotation_reference_frame[1];
    matrix_rotation_reference_frame.e02 = user_data.rotation_reference_frame[2];
    matrix_rotation_reference_frame.e10 = user_data.rotation_reference_frame[3];
    matrix_rotation_reference_frame.e11 = user_data.rotation_reference_frame[4];
    matrix_rotation_reference_frame.e12 = user_data.rotation_reference_frame[5];
    matrix_rotation_reference_frame.e20 = user_data.rotation_reference_frame[6];
    matrix_rotation_reference_frame.e21 = user_data.rotation_reference_frame[7];
    matrix_rotation_reference_frame.e22 = user_data.rotation_reference_frame[8];

    // Check diagonal to determine if the matrix is different, the rest of the values should be 0
    // There is no method to compare matrices directly
    if (current_rotation_reference_frame.e00 != matrix_rotation_reference_frame.e00
      || current_rotation_reference_frame.e11 != matrix_rotation_reference_frame.e11
      || current_rotation_reference_frame.e22 != matrix_rotation_reference_frame.e22)
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Current rotation reference frame is different from the desired one: " << matrix_rotation_reference_frame);
      vs.writeReferenceFrameRotation(matrix_rotation_reference_frame, true);
      current_rotation_reference_frame = vs.readReferenceFrameRotation();
      RCLCPP_INFO_STREAM(node->get_logger(), "New rotation reference frame: " << current_rotation_reference_frame);
      RCLCPP_INFO_STREAM(node->get_logger(), "Restarting device to save new reference frame");
      vs.writeSettings(true);
      vs.reset();
    }

  }

  // Register async callback function
  vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

  // Write bias compensation
  if (user_data.acc_bias_enable) {
    setHorizontalSrv = node->create_service<std_srvs::srv::Trigger>(
      "~/set_acc_bias", std::bind(set_horizontal, std::placeholders::_1, std::placeholders::_2, &vs, &SensorImuRate, &user_data.set_acc_bias_seconds));
    resetHorizontalSrv = node->create_service<std_srvs::srv::Trigger>(
      "~/reset_acc_bias", std::bind(reset_horizontal, std::placeholders::_1, std::placeholders::_2, &vs));
  }
  resetDeviceSrv = node->create_service<std_srvs::srv::Trigger>(
    "~/reset_device", std::bind(reset_device, std::placeholders::_1, std::placeholders::_2, &vs));
  tareDeviceSrv = node->create_service<std_srvs::srv::Trigger>(
    "~/tare_device", std::bind(tare_device, std::placeholders::_1, std::placeholders::_2, &vs));

  // You spin me right round, baby
  // Right round like a record, baby
  // Right round round round
  rclcpp::spin(node);

  // Node has been terminated
  vs.unregisterAsyncPacketReceivedHandler();
  RCLCPP_INFO(node->get_logger(), "Unregisted the packet received handler");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  vs.disconnect();
  RCLCPP_INFO(node->get_logger(), "%s is disconnected successfully", mn.c_str());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return 0;
}

//Helper function to create IMU message
bool fill_imu_message(
  sensor_msgs::msg::Imu & msgIMU, vn::sensors::CompositeData & cd, rclcpp::Time & time,
  UserData * user_data)
{
  msgIMU.header.stamp = time;
  msgIMU.header.frame_id = user_data->frame_id;

  if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration()) {
    vec4f q = cd.quaternion();
    vec3f ar = cd.angularRate();
    vec3f al = cd.acceleration();

    if (cd.hasAttitudeUncertainty()) {
      vec3f orientationStdDev = cd.attitudeUncertainty();
      msgIMU.orientation_covariance[0] =
        pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Roll
      msgIMU.orientation_covariance[4] =
        pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians Pitch
      msgIMU.orientation_covariance[8] =
        pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Yaw
    }

    if (not ValidateQuaternion(q) or not ValidateVector(ar) or not ValidateVector(al))
    {
        invalid_data++;
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1, "Invalid data (%d until now). Orientation: %f, %f, %f, %f. Angular velocity: %f, %f, %f. Linear Acceleration: %f, %f, %f",
                    invalid_data, q[0], q[1], q[2], q[3],
                    ar[0], ar[1], ar[2], al[0], al[1], al[2]);

        // Shutdown node if more than max_invalid_packets are received consecutively
        if ((max_invalid_packets != -1) && (invalid_data >= max_invalid_packets))
        {
          rclcpp::shutdown();
        }

        return false;
    }
    else
    {
      invalid_data = 0;
      //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
      if (user_data->tf_ned_to_enu) {
        // If we want the orientation to be based on the reference label on the imu
        tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
        geometry_msgs::msg::Quaternion quat_msg;

        if (user_data->frame_based_enu) {
          // Create a rotation from NED -> ENU
          tf2::Quaternion q_rotate;
          q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
          // Apply the NED to ENU rotation such that the coordinate frame matches
          tf2_quat = q_rotate * tf2_quat;
          quat_msg = tf2::toMsg(tf2_quat);

          // Since everything is in the normal frame, no flipping required
          msgIMU.angular_velocity.x = ar[0];
          msgIMU.angular_velocity.y = ar[1];
          msgIMU.angular_velocity.z = ar[2];

          msgIMU.linear_acceleration.x = al[0];
          msgIMU.linear_acceleration.y = al[1];
          msgIMU.linear_acceleration.z = al[2];
        } else {
          // put into ENU - swap X/Y, invert Z
          quat_msg.x = q[1];
          quat_msg.y = q[0];
          quat_msg.z = -q[2];
          quat_msg.w = q[3];

          // Flip x and y then invert z
          msgIMU.angular_velocity.x = ar[1];
          msgIMU.angular_velocity.y = ar[0];
          msgIMU.angular_velocity.z = -ar[2];
          // Flip x and y then invert z
          msgIMU.linear_acceleration.x = al[1];
          msgIMU.linear_acceleration.y = al[0];
          msgIMU.linear_acceleration.z = -al[2];

          if (cd.hasAttitudeUncertainty()) {
            vec3f orientationStdDev = cd.attitudeUncertainty();
            msgIMU.orientation_covariance[0] =
              pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians pitch
            msgIMU.orientation_covariance[4] =
              pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Roll
            msgIMU.orientation_covariance[8] =
              pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Yaw
          }
        }

        msgIMU.orientation = quat_msg;
      }
      else if (user_data->tf_ned_to_nwu) {
        tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
        geometry_msgs::msg::Quaternion quat_msg;

        if (user_data->frame_based_nwu) {
          // Create a rotation from NED -> NWU
          tf2::Quaternion q_rotate;
          q_rotate.setRPY(M_PI, 0.0, 0.0);
          // Apply the NED to ENU rotation such that the coordinate frame matches
          tf2_quat = q_rotate * tf2_quat;
          quat_msg = tf2::toMsg(tf2_quat);

          // Since everything is in the normal frame, no flipping required
          msgIMU.angular_velocity.x = ar[0];
          msgIMU.angular_velocity.y = ar[1];
          msgIMU.angular_velocity.z = ar[2];

          msgIMU.linear_acceleration.x = al[0];
          msgIMU.linear_acceleration.y = al[1];
          msgIMU.linear_acceleration.z = al[2];
        } else {
          // put into NWU
          quat_msg.x = q[0];
          quat_msg.y = -q[1];
          quat_msg.z = -q[2];
          quat_msg.w = q[3];

          // Invert y and z
          msgIMU.angular_velocity.x = ar[0];
          msgIMU.angular_velocity.y = -ar[1];
          msgIMU.angular_velocity.z = -ar[2];
          // Invert y and z
          msgIMU.linear_acceleration.x = al[0];
          msgIMU.linear_acceleration.y = -al[1];
          msgIMU.linear_acceleration.z = -al[2];

          if (cd.hasAttitudeUncertainty()) {
            vec3f orientationStdDev = cd.attitudeUncertainty();
            msgIMU.orientation_covariance[0] =
              pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians roll
            msgIMU.orientation_covariance[4] =
              pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians pitch
            msgIMU.orientation_covariance[8] =
              pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Yaw
          }
        }
        msgIMU.orientation = quat_msg;
      } else {
        msgIMU.orientation.x = q[0];
        msgIMU.orientation.y = q[1];
        msgIMU.orientation.z = q[2];
        msgIMU.orientation.w = q[3];

        msgIMU.angular_velocity.x = ar[0];
        msgIMU.angular_velocity.y = ar[1];
        msgIMU.angular_velocity.z = ar[2];
        msgIMU.linear_acceleration.x = al[0];
        msgIMU.linear_acceleration.y = al[1];
        msgIMU.linear_acceleration.z = al[2];
      }
      // Covariances pulled from parameters
      std::copy(user_data->orientation_covariance.begin(),
        user_data->orientation_covariance.end(),
        msgIMU.orientation_covariance.begin());
      std::copy(user_data->angular_vel_covariance.begin(),
        user_data->angular_vel_covariance.end(),
        msgIMU.angular_velocity_covariance.begin());
      std::copy(user_data->linear_accel_covariance.begin(),
        user_data->linear_accel_covariance.end(),
        msgIMU.linear_acceleration_covariance.begin());
    }
    return true;
  }
  RCLCPP_WARN(node->get_logger(),  "IMU invalid data, discarding message");
  return false;
}

//Helper function to create magnetic field message
void fill_mag_message(
  sensor_msgs::msg::MagneticField & msgMag, vn::sensors::CompositeData & cd, rclcpp::Time & time,
  UserData * user_data)
{
  msgMag.header.stamp = time;
  msgMag.header.frame_id = user_data->frame_id;

  // Magnetic Field
  if (cd.hasMagnetic()) {
    vec3f mag = cd.magnetic();
    msgMag.magnetic_field.x = mag[0];
    msgMag.magnetic_field.y = mag[1];
    msgMag.magnetic_field.z = mag[2];
  }
}

//Helper function to create gps message
void fill_gps_message(
  sensor_msgs::msg::NavSatFix & msgGPS, vn::sensors::CompositeData & cd, rclcpp::Time & time,
  UserData * user_data)
{
  msgGPS.header.stamp = time;
  msgGPS.header.frame_id = user_data->frame_id;

  if (cd.hasPositionEstimatedLla()) {
    vec3d lla = cd.positionEstimatedLla();

    msgGPS.latitude = lla[0];
    msgGPS.longitude = lla[1];
    msgGPS.altitude = lla[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      msgGPS.position_covariance[0] = posVariance;  // East position variance
      msgGPS.position_covariance[4] = posVariance;  // North position vaciance
      msgGPS.position_covariance[8] = posVariance;  // Up position variance

      // mark gps fix as not available if the outputted standard deviation is 0
      if (cd.positionUncertaintyEstimated() != 0.0) {
        // Position available
        msgGPS.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      } else {
        // position not detected
        msgGPS.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }

      // add the type of covariance to the gps message
      msgGPS.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      msgGPS.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
  }
}

//Helper function to create odometry message
void fill_odom_message(
  nav_msgs::msg::Odometry & msgOdom, vn::sensors::CompositeData & cd, rclcpp::Time & time,
  UserData * user_data)
{
  msgOdom.header.stamp = time;
  msgOdom.child_frame_id = user_data->frame_id;
  msgOdom.header.frame_id = user_data->map_frame_id;

  if (cd.hasPositionEstimatedEcef()) {
    // add position as earth fixed frame
    vec3d pos = cd.positionEstimatedEcef();

    if (!user_data->initial_position_set) {
      RCLCPP_INFO(node->get_logger(),  "Set initial position to %f %f %f", pos[0], pos[1], pos[2]);
      user_data->initial_position_set = true;
      user_data->initial_position.x = pos[0];
      user_data->initial_position.y = pos[1];
      user_data->initial_position.z = pos[2];
    }

    msgOdom.pose.pose.position.x = pos[0] - user_data->initial_position[0];
    msgOdom.pose.pose.position.y = pos[1] - user_data->initial_position[1];
    msgOdom.pose.pose.position.z = pos[2] - user_data->initial_position[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      msgOdom.pose.covariance[0] = posVariance;   // x-axis position variance
      msgOdom.pose.covariance[7] = posVariance;   // y-axis position vaciance
      msgOdom.pose.covariance[14] = posVariance;  // z-axis position variance
    }
  }

  if (cd.hasQuaternion()) {
    vec4f q = cd.quaternion();

    if (!user_data->tf_ned_to_enu && !user_data->tf_ned_to_nwu) {
      // output in NED frame
      msgOdom.pose.pose.orientation.x = q[0];
      msgOdom.pose.pose.orientation.y = q[1];
      msgOdom.pose.pose.orientation.z = q[2];
      msgOdom.pose.pose.orientation.w = q[3];
    } else if (user_data->tf_ned_to_enu && user_data->frame_based_enu) {
      // standard conversion from NED to ENU frame
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      // Create a rotation from NED -> ENU
      tf2::Quaternion q_rotate;
      q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
      // Apply the NED to ENU rotation such that the coordinate frame matches
      tf2_quat = q_rotate * tf2_quat;
      msgOdom.pose.pose.orientation = tf2::toMsg(tf2_quat);
    } else if (user_data->tf_ned_to_enu && !user_data->frame_based_enu) {
      // alternative method for conversion to ENU frame (leads to another result)
      // put into ENU - swap X/Y, invert Z
      msgOdom.pose.pose.orientation.x = q[1];
      msgOdom.pose.pose.orientation.y = q[0];
      msgOdom.pose.pose.orientation.z = -q[2];
      msgOdom.pose.pose.orientation.w = q[3];
    } else if (user_data->tf_ned_to_nwu && user_data->frame_based_nwu) {
      // standard conversion from NED to ENU frame
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      // Create a rotation from NED -> ENU
      tf2::Quaternion q_rotate;
      q_rotate.setRPY(M_PI, 0.0, 0.0);
      // Apply the NED to ENU rotation such that the coordinate frame matches
      tf2_quat = q_rotate * tf2_quat;
      msgOdom.pose.pose.orientation = tf2::toMsg(tf2_quat);
    } else if (user_data->tf_ned_to_nwu && !user_data->frame_based_nwu) {
      // alternative method for conversion to ENU frame (leads to another result)
      // put into ENU - swap X/Y, invert Z
      msgOdom.pose.pose.orientation.x = q[0];
      msgOdom.pose.pose.orientation.y = -q[1];
      msgOdom.pose.pose.orientation.z = -q[2];
      msgOdom.pose.pose.orientation.w = q[3];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasAttitudeUncertainty()) {
      vec3f orientationStdDev = cd.attitudeUncertainty();
      // convert the standard deviation values from all three axis from degrees to radiant and calculate the variances from these (squared), which are assigned to the covariance matrix.
      if ( (!user_data->tf_ned_to_enu && !user_data->tf_ned_to_nwu) || user_data->frame_based_enu || user_data->frame_based_nwu) {
        // standard assignment of variance values for NED frame and conversion to ENU frame by rotation
        msgOdom.pose.covariance[21] = pow(orientationStdDev[0] * M_PI / 180, 2);  // roll variance
        msgOdom.pose.covariance[28] = pow(orientationStdDev[1] * M_PI / 180, 2);  // pitch variance
        msgOdom.pose.covariance[35] = pow(orientationStdDev[2] * M_PI / 180, 2);  // yaw variance
      } else if (user_data->tf_ned_to_enu && !user_data->frame_based_enu){
        // variance assignment for conversion by swapping and inverting (not frame_based_enu or frame_based_nwu)
        // TODO not supported yet
      } else if (user_data->tf_ned_to_nwu && !user_data->frame_based_nwu) {
        // variance assignment for conversion by swapping and inverting (not frame_based_enu or frame_based_nwu)
        // TODO not supported yet
      }
    }
  }

  // Add the velocity in the body frame (frame_id) to the message
  if (cd.hasVelocityEstimatedBody()) {
    vec3f vel = cd.velocityEstimatedBody();

    if ( (!user_data->tf_ned_to_enu && !user_data->tf_ned_to_nwu) || user_data->frame_based_enu || user_data->frame_based_nwu) {
      // standard assignment of values for NED frame and conversion to ENU or NWU frame by rotation
      msgOdom.twist.twist.linear.x = vel[0];
      msgOdom.twist.twist.linear.y = vel[1];
      msgOdom.twist.twist.linear.z = vel[2];
    } else if (user_data->tf_ned_to_enu && !user_data->frame_based_enu) {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      msgOdom.twist.twist.linear.x = vel[1];
      msgOdom.twist.twist.linear.y = vel[0];
      msgOdom.twist.twist.linear.z = -vel[2];
    } else if (user_data->tf_ned_to_nwu && !user_data->frame_based_nwu) {
      // value assignment for conversion by swapping and inverting (not frame_based_nwu)
      // Invert y and z
      msgOdom.twist.twist.linear.x = vel[0];
      msgOdom.twist.twist.linear.y = -vel[1];
      msgOdom.twist.twist.linear.z = -vel[2];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasVelocityUncertaintyEstimated()) {
      double velVariance = pow(cd.velocityUncertaintyEstimated(), 2);
      msgOdom.twist.covariance[0] = velVariance;   // x-axis velocity variance
      msgOdom.twist.covariance[7] = velVariance;   // y-axis velocity vaciance
      msgOdom.twist.covariance[14] = velVariance;  // z-axis velocity variance

      // set velocity variances to a high value if no data is available (this is the case at startup during INS is initializing)
      if (
        msgOdom.twist.twist.linear.x == 0 && msgOdom.twist.twist.linear.y == 0 &&
        msgOdom.twist.twist.linear.z == 0 && msgOdom.twist.covariance[0] == 0 &&
        msgOdom.twist.covariance[7] == 0 && msgOdom.twist.covariance[14] == 0) {
        msgOdom.twist.covariance[0] = 200;
        msgOdom.twist.covariance[7] = 200;
        msgOdom.twist.covariance[15] = 200;
      }
    }
  }

  if (cd.hasAngularRate()) {
    vec3f ar = cd.angularRate();

    if ( (!user_data->tf_ned_to_enu && !user_data->tf_ned_to_nwu) || user_data->frame_based_enu || user_data->frame_based_nwu) {
      // standard assignment of values for NED frame and conversion to ENU or NWU frame by rotation
      msgOdom.twist.twist.angular.x = ar[0];
      msgOdom.twist.twist.angular.y = ar[1];
      msgOdom.twist.twist.angular.z = ar[2];
    } else if (user_data->tf_ned_to_enu && !user_data->frame_based_enu) {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      msgOdom.twist.twist.angular.x = ar[1];
      msgOdom.twist.twist.angular.y = ar[0];
      msgOdom.twist.twist.angular.z = -ar[2];
    } else if (user_data->tf_ned_to_nwu && !user_data->frame_based_nwu) {
      // value assignment for conversion by swapping and inverting (not frame_based_nwu)
      // Invert y and z
      msgOdom.twist.twist.angular.x = ar[0];
      msgOdom.twist.twist.angular.y = -ar[1];
      msgOdom.twist.twist.angular.z = -ar[2];
    }

    // add covariance matrix of the measured angular rate to odom message.
    // go through matrix rows
    for (int row = 0; row < 3; row++) {
      // go through matrix columns
      for (int col = 0; col < 3; col++) {
        // Target matrix has 6 rows and 6 columns, source matrix has 3 rows and 3 columns. The covariance values are put into the fields (3, 3) to (5, 5) of the destination matrix.
        msgOdom.twist.covariance[(row + 3) * 6 + (col + 3)] =
          user_data->angular_vel_covariance[row * 3 + col];
      }
    }
  }
}

//Helper function to create temperature message
void fill_temp_message(
  sensor_msgs::msg::Temperature & msgTemp, vn::sensors::CompositeData & cd, rclcpp::Time & time,
  UserData * user_data)
{
  msgTemp.header.stamp = time;
  msgTemp.header.frame_id = user_data->frame_id;
  if (cd.hasTemperature()) {
    float temp = cd.temperature();
    msgTemp.temperature = temp;
  }
}

//Helper function to create pressure message
void fill_pres_message(
  sensor_msgs::msg::FluidPressure & msgPres, vn::sensors::CompositeData & cd, rclcpp::Time & time,
  UserData * user_data)
{
  msgPres.header.stamp = time;
  msgPres.header.frame_id = user_data->frame_id;
  if (cd.hasPressure()) {
    float pres = cd.pressure();
    msgPres.fluid_pressure = pres;
  }
}

//Helper function to create ins message
void fill_ins_message(
  vectornav::msg::Ins & msgINS, vn::sensors::CompositeData & cd, rclcpp::Time & time, UserData * user_data)
{
  msgINS.header.stamp = time;
  msgINS.header.frame_id = user_data->frame_id;

  if (cd.hasInsStatus()) {
    InsStatus insStatus = cd.insStatus();
    msgINS.ins_status = static_cast<uint16_t>(insStatus);
  }

  if (cd.hasTow()) {
    msgINS.time = cd.tow();
  }

  if (cd.hasWeek()) {
    msgINS.week = cd.week();
  }

  if (cd.hasTimeUtc()) {
    TimeUtc utcTime = cd.timeUtc();
    char * utcTimeBytes = reinterpret_cast<char *>(&utcTime);
    //msgINS.utcTime bytes are in Little Endian Byte Order
    std::memcpy(&msgINS.utc_time, utcTimeBytes, 8);
  }

  if (cd.hasYawPitchRoll()) {
    vec3f rpy = cd.yawPitchRoll();
    msgINS.yaw = rpy[0];
    msgINS.pitch = rpy[1];
    msgINS.roll = rpy[2];
  }

  if (cd.hasPositionEstimatedLla()) {
    vec3d lla = cd.positionEstimatedLla();
    msgINS.latitude = lla[0];
    msgINS.longitude = lla[1];
    msgINS.altitude = lla[2];
  }

  if (cd.hasVelocityEstimatedNed()) {
    vec3f nedVel = cd.velocityEstimatedNed();
    msgINS.ned_vel_x = nedVel[0];
    msgINS.ned_vel_y = nedVel[1];
    msgINS.ned_vel_z = nedVel[2];
  }

  if (cd.hasAttitudeUncertainty()) {
    vec3f attUncertainty = cd.attitudeUncertainty();
    msgINS.att_uncertainty[0] = attUncertainty[0];
    msgINS.att_uncertainty[1] = attUncertainty[1];
    msgINS.att_uncertainty[2] = attUncertainty[2];
  }

  if (cd.hasPositionUncertaintyEstimated()) {
    msgINS.pos_uncertainty = cd.positionUncertaintyEstimated();
  }

  if (cd.hasVelocityUncertaintyEstimated()) {
    msgINS.vel_uncertainty = cd.velocityUncertaintyEstimated();
  }
}

static rclcpp::Time get_time_stamp(
  vn::sensors::CompositeData & cd, UserData * user_data, const rclcpp::Time & ros_time)
{
  if (!cd.hasTimeStartup() || !user_data->adjust_ros_timestamp) {
    return (ros_time);  // don't adjust timestamp
  }

  const double sensor_time = cd.timeStartup() * 1e-9;  // time in seconds

  if (user_data->average_time_difference == 0) {       // first call
    user_data->ros_start_time = ros_time;
    user_data->average_time_difference = static_cast<double>(-sensor_time);
    user_data->last_sensor_time = sensor_time;
  }

  if (!validateSensorTimestamp(sensor_time, user_data))
  {
    return rclcpp::Time(0);
  }


  // difference between node startup and current ROS time
  const double ros_dt = (ros_time - user_data->ros_start_time).seconds();

  // Do not use ros_dt to calculate average_time_difference if it is smaller than last measurement
  if (ros_dt > user_data->biggest_ros_dt)
  {
    // difference between elapsed ROS time and time since sensor startup
    const double dt = ros_dt - sensor_time;
    // compute exponential moving average
    const double alpha = 0.001;  // average over rougly 1000 samples
    user_data->average_time_difference =
      user_data->average_time_difference * (1.0 - alpha) + alpha * dt;
    user_data->biggest_ros_dt = ros_dt;
  }
  else
  {
    RCLCPP_WARN(node->get_logger(),  "WARNING: ros_dt: %f is smaller than biggest_ros_dt: %f."
      "This ros_dt will not be used to calculate average_time_difference", ros_dt, user_data->biggest_ros_dt);
  }

  // adjust sensor time by average difference to ROS time
  const rclcpp::Time adj_time =
    user_data->ros_start_time + rclcpp::Duration(std::chrono::microseconds(static_cast<uint64_t>(1e6*(user_data->average_time_difference + sensor_time))));
  return (adj_time);
}

//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index)
{
  // package counter to calculate strides
  static unsigned long long pkg_count = 0;

  // evaluate time first, to have it as close to the measurement time as possible
  const rclcpp::Time ros_time = node->get_clock()->now();

  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  UserData * user_data = static_cast<UserData *>(userData);
  rclcpp::Time time = get_time_stamp(cd, user_data, ros_time);

  // Only publish if timestamp did not go back in time
  if (newest_timestamp < time)
  {
    newest_timestamp = time;
    // IMU
    std::unique_lock<std::mutex> lock(mtx_samples);
    if (((pkg_count % user_data->imu_stride) == 0 && pubIMU->get_subscription_count() > 0) || take_samples) {
      sensor_msgs::msg::Imu msgIMU;
      if (fill_imu_message(msgIMU, cd, time, user_data) == true)
      {
        if ((pkg_count % user_data->imu_stride) == 0)
          pubIMU->publish(msgIMU);
        if (take_samples)
          samples.push_back({msgIMU.linear_acceleration.x, msgIMU.linear_acceleration.y, msgIMU.linear_acceleration.z});
      }
    }
    lock.unlock();

    if ((pkg_count % user_data->output_stride) == 0) {
      // Magnetic Field
      if (pubMag->get_subscription_count() > 0) {
        sensor_msgs::msg::MagneticField msgMag;
        fill_mag_message(msgMag, cd, time, user_data);
        pubMag->publish(msgMag);
      }

      // Temperature
      if (pubTemp->get_subscription_count() > 0) {
        sensor_msgs::msg::Temperature msgTemp;
        fill_temp_message(msgTemp, cd, time, user_data);
        pubTemp->publish(msgTemp);
      }

      // Barometer
      if (pubPres->get_subscription_count() > 0) {
        sensor_msgs::msg::FluidPressure msgPres;
        fill_pres_message(msgPres, cd, time, user_data);
        pubPres->publish(msgPres);
      }

      // GPS
      if (
        user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
        pubGPS->get_subscription_count() > 0) {
        sensor_msgs::msg::NavSatFix msgGPS;
        fill_gps_message(msgGPS, cd, time, user_data);
        pubGPS->publish(msgGPS);
      }

      // Odometry
      if (
        user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
        pubOdom->get_subscription_count() > 0) {
        nav_msgs::msg::Odometry msgOdom;
        fill_odom_message(msgOdom, cd, time, user_data);
        pubOdom->publish(msgOdom);
      }

      // INS
      if (
        user_data->device_family != VnSensor::Family::VnSensor_Family_Vn100 &&
        pubIns->get_subscription_count() > 0) {
        vectornav::msg::Ins msgINS;
        fill_ins_message(msgINS, cd, time, user_data);
        pubIns->publish(msgINS);
      }
    }
  }
  else
  {
    RCLCPP_WARN(node->get_logger(),  "IMU message filtered, timestamp went back in time");
  }
  pkg_count += 1;
}

bool ValidateQuaternion(vec4f q)
{
    return std::isfinite(q[0]) and std::isfinite(q[1]) and std::isfinite(q[2]) and std::isfinite(q[3])
    and (std::abs(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] - 1.0f) < 0.01)
    and !(q[0] == 0 && q[1] == 0 && q[2] == 0 && q[3] == 0);
}

bool ValidateVector(vec3f v)
{
    return std::isfinite(v[0]) and std::isfinite(v[1]) and std::isfinite(v[2]);
}

bool validateSensorTimestamp(const double sensor_time, UserData * user_data)
{
  bool isValid = true;

  // Do not calcuate timestamp if difference between current and previous timestamp is higher than expected
  if (std::abs(sensor_time - user_data->last_sensor_time) > user_data->maximum_imu_timestamp_difference)
  {
    RCLCPP_WARN(node->get_logger(),  "WARNING: difference between sensor_time: %f and last_sensor_time: %f is bigger than "
      "maximum_imu_timestamp_difference: %f. Returning an invalid timestamp to reject "
       "this measurement", sensor_time, user_data->last_sensor_time, user_data->maximum_imu_timestamp_difference);
    isValid = false;
  }

  user_data->last_sensor_time = sensor_time;

  if (isValid)
  {
    // Do not calcuate timestamp nor update newest_sensor_time if sensor_time is smaller than last measurement
    if (sensor_time < user_data->newest_sensor_time)
    {
      RCLCPP_WARN(node->get_logger(),  "WARNING: sensor_time: %f is smaller than newest_sensor_time: %f."
        "Returning an invalid timestamp to reject this measurement", sensor_time, user_data->newest_sensor_time);
      isValid = false;
    }
    else
    {
      user_data->newest_sensor_time = sensor_time;
    }
  }

  return isValid;
}
