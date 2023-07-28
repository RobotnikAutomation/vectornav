#include <vectornav/vectornav.hpp>

#include <tf2/LinearMath/Transform.h>

// Assure that the serial port is set to async low latency in order to reduce delays and package pilup.
// These changes will stay effective until the device is unplugged
#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
bool optimize_serial_communication(std::string const & portName)
{
  int portFd = -1;

  ROS_DEBUG_NAMED("serial_optimizer", "Open port for optimization");
  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    ROS_WARN_NAMED("serial_optimizer", "Can't open port for optimization");
    return false;
  }

  ROS_DEBUG_NAMED("serial_optimizer", "Setting port to ASYNCY_LOW_LATENCY");
  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ROS_DEBUG_NAMED("serial_optimizer", "Closing port");
  ::close(portFd);
  return true;
}
#elif
bool optimize_serial_communication(str::string portName) {
  ROS_WARN_NAMED("serial_optimizer",
                 "Serial port low latency optimization not implemented on build platform");
  return true;
}
#endif

namespace vectornav
{

Vectornav::Vectornav(ros::NodeHandle & nh, ros::NodeHandle & pnh)
: nh_{nh}, pnh_{pnh}, params_{}
, vs_{}
{
  ROS_INFO_NAMED("vectornav", "Initializing vectornav device");

  // Read parameters from the parameter server
  ROS_DEBUG_NAMED("vectornav", "Reading parameters from server");
  read_parameters();

  // Optimize serial low latencty communication
  ROS_DEBUG_NAMED("vectornav", "Optimize serial low latencty communication");
  optimize_serial_communication(params_.port);

  // Connect to the device
  ROS_INFO_NAMED("vectornav", "Connecting to: %s @ %d", params_.port.c_str(), params_.baudrate);
  connect_device(params_.port, params_.baudrate);

  ROS_DEBUG_NAMED("vectornav", "Advertising topics");
  advertise_topics();

  ROS_DEBUG_NAMED("vectornav", "Advertising services");
  advertise_services();

  ROS_DEBUG_NAMED("vectornav", "Configuring device");
  configure_device();

  ROS_DEBUG_NAMED("vectornav", "Finished initialization");
}

void Vectornav::spin()
{
  while (ros::ok())
    ros::spin();  // Need to make sure we disconnect properly. Check if all ok.

  disconnect_device();
}

void Vectornav::read_parameters()
{
  pnh_.param<std::string>("map_frame_id",
                          params_.map_frame_id,
                          params_.map_frame_id);
  pnh_.param<std::string>("frame_id",
                          params_.imu_frame_id,
                          params_.imu_frame_id);
  pnh_.param<bool>("tf_ned_to_enu",
                   params_.tf_ned_to_enu,
                   params_.tf_ned_to_enu);
  pnh_.param<bool>("frame_based_enu",
                   params_.frame_based_enu,
                   params_.frame_based_enu);
  pnh_.param<bool>("tf_ned_to_nwu",
                   params_.tf_ned_to_nwu,
                   params_.tf_ned_to_nwu);
  pnh_.param<bool>("frame_based_nwu",
                   params_.frame_based_nwu,
                   params_.frame_based_nwu);
  pnh_.param<bool>("adjust_ros_timestamp",
                   params_.adjust_ros_timestamp,
                   params_.adjust_ros_timestamp);
  pnh_.param<int>("async_output_rate",
                  params_.async_output_rate,
                  params_.async_output_rate);
  pnh_.param<int>("imu_output_rate",
                  params_.imu_output_rate,
                  params_.imu_output_rate);
  pnh_.param<std::string>("serial_port",
                          params_.port,
                          params_.port);
  pnh_.param<int>("serial_baud",
                  params_.baudrate,
                  params_.baudrate);
  pnh_.param<int>("fixed_imu_rate",
                  params_.fixed_imu_rate,
                  params_.fixed_imu_rate);
  pnh_.param<int>("max_invalid_packets",
                  params_.max_invalid_packets,
                  params_.max_invalid_packets);

  pnh_.param<bool>("acc_bias_enable", params_.acc_bias_enable, false);
  pnh_.param<double>("set_acc_bias_seconds", params_.set_acc_bias_seconds, 2.5);

  pnh_.getParam("linear_accel_covariance", params_.linear_accel_covariance);
  pnh_.getParam("angular_vel_covariance", params_.angular_vel_covariance);
  pnh_.getParam("orientation_covariance", params_.orientation_covariance); // Unused?
  has_rotation_reference_frame_ = pnh_.getParam("rotation_reference_frame", params_.rotation_reference_frame);
}

void Vectornav::advertise_topics()
{
  pub_imu_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1000);
  pub_mag_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);
  pub_temp_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 1000);
  pub_pres_ = nh_.advertise<sensor_msgs::FluidPressure>("imu/atm_pressure", 1000);

  // Filter topics (not supported by VN-100)
  if (device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100) {
    pub_gps_ = nh_.advertise<sensor_msgs::NavSatFix>("imu/global_position/raw/fix", 1000);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("imu/odom", 1000);
    pub_ins_ = nh_.advertise<vectornav::Ins>("imu/INS", 1000);
  }
}

void Vectornav::advertise_services()
{
  if (params_.acc_bias_enable) {
    // Write bias compensation
    srv_set_horizontal_ = pnh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "set_acc_bias", std::bind(&Vectornav::set_horizontal, this, std::placeholders::_1, std::placeholders::_2));
    // Reset bias compensation
    srv_reset_horizontal_ = pnh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "reset_acc_bias", std::bind(&Vectornav::reset_horizontal, this, std::placeholders::_1, std::placeholders::_2));
  }
  // Filter unnecessary services (not supported by VN-100)
  if (device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100) {
    srv_reset_odom_ = nh_.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
      "reset_odom", std::bind(&Vectornav::reset_odometry, this, std::placeholders::_1, std::placeholders::_2));
  }
}
bool Vectornav::reset_horizontal(std_srvs::Trigger::Request const & req, std_srvs::Trigger::Response & res)
{
  std::unique_lock<std::mutex> service_lock(service_acc_bias_mtx_);
  ROS_INFO("Reset to factory new acceleration bias");

  vn::math::mat3f const gain {1., 0., 0.,
                              0., 1., 0.,
                              0., 0., 1.};
  vs_.writeAccelerationCompensation(gain, {0., 0., 0.}, true);
  vs_.writeSettings(true);

  res.success = true;
  res.message = "Bias register set to zero. Please, reset vectornav hardware to avoid angular velocity.";
  ROS_INFO("Done.");
  return true;
}

bool Vectornav::set_horizontal(std_srvs::Trigger::Request const & req, std_srvs::Trigger::Response & res)
{
  std::unique_lock<std::mutex> sample_lock(mtx_samples_, std::defer_lock);
  std::unique_lock<std::mutex> service_lock(service_acc_bias_mtx_);
  ROS_INFO_NAMED("vectornav", "Set acceleration bias to zero (0.0, 0.0, 9.81)");
  vn::math::mat3f const gain {1., 0., 0.,
                              0., 1., 0.,
                              0., 0., 1.};

  sample_lock.lock();
    samples_.clear();
    samples_.reserve(static_cast<size_t>(params_.set_acc_bias_seconds * params_.imu_output_rate * 1.5));
    take_samples_ = true;
    auto const start = ros::Time::now();
  sample_lock.unlock();

  ros::Duration(params_.set_acc_bias_seconds).sleep();

  sample_lock.lock();
    auto const end = ros::Time::now();
    take_samples_ = false;

    // Calculate mean of samples
    double bias_x{0.}, bias_y{0.}, bias_z{0.};
    for (auto const & sample : samples_) {
      bias_x += sample.x; bias_y += sample.y; bias_z += sample.z;
    }
    bias_x /= samples_.size(); bias_y /= samples_.size(); bias_z /= samples_.size();

    // Calculate covariance of samples
    double covariance_x{0.}, covariance_y{0.}, covariance_z{0.};
    for (auto const & sample : samples_) {
      covariance_x += (sample.x - bias_x) * (sample.x - bias_x);
      covariance_y += (sample.y - bias_y) * (sample.y - bias_y);
      covariance_z += (sample.z - bias_z) * (sample.z - bias_z);
    }
    covariance_x /= samples_.size(); covariance_y /= samples_.size(); covariance_z /= samples_.size();
  sample_lock.unlock();

  auto const curr { vs_.readAccelerationCompensation() };

  vn::math::vec3f const bias {curr.b.x-static_cast<float>(bias_x),
                              curr.b.y+static_cast<float>(bias_y),
                              curr.b.z-static_cast<float>(bias_z-9.80665)};

  if (samples_.size() < 10) {
    ROS_ERROR("Not enough samples taken (<10). Aborting.");
    res.message = "Not enough samples taken (<10). Aborting.";
    res.success = false;
  } else {
    ROS_INFO_NAMED("vectornav", "Applying bias correction to vectornav:");
    ROS_INFO_NAMED("vectornav", " - Samples taked: %d (%.2lfs)", samples_.size(), (end - start).toSec());
    ROS_INFO_NAMED("vectornav", " - Bias:       [x: %7.4lf, y: %7.4lf, z: %7.4lf]", bias.x, bias.y, bias.z);
    ROS_INFO_NAMED("vectornav", " - Covariance: [x: %7.4lf, y: %7.4lf, z: %7.4lf]", covariance_x, covariance_y, covariance_z);
    res.message = "Applying bias correction to vectornav, see log for more info. Please, reset vectornav hardware to avoid angular velocity.";
    res.success = true;
    vs_.writeAccelerationCompensation(gain, {bias.x, bias.y, bias.z}, true);
    vs_.writeSettings(true);
    ROS_INFO_NAMED("vectornav", "Done.");
  }

  return true;
}

bool Vectornav::reset_odometry(std_srvs::Empty::Request const & req, std_srvs::Empty::Response & res)
{
  ROS_INFO_NAMED("vectornav", "Reset odometry callback received. Resetting odometry.");
  initial_position_set_ = false;
  return true;
}

void Vectornav::connect_device(std::string const & port, int baudrate)
{
  // Default baudrate variable
  int defaultBaudrate;

  // Run through all of the acceptable baud rates until we are connected
  // Looping in case someone has changed the default
  bool baudSet = false;

  // Lets add the set baudrate to the top of the list, so that it will try
  // to connect with that value first (speed initialization up)
  std::vector<unsigned int> supportedBaudrates = vs_.supportedBaudrates();
  supportedBaudrates.insert(supportedBaudrates.begin(), baudrate);

  while (!baudSet) {
    // Make this variable only accessible in the while loop
    static int i = 0;
    defaultBaudrate = supportedBaudrates[i];
    ROS_DEBUG_NAMED("vectornav", "Trying to connect with baudrate: %d", defaultBaudrate);
    // Default response was too low and retransmit time was too long by default.
    // They would cause errors
    vs_.setResponseTimeoutMs(1000);  // Wait for up to 1000 ms for response
    vs_.setRetransmitDelayMs(50);    // Retransmit every 50 ms

    // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400, 460800, 921600
    // Data sheet says 128000 is a valid baud rate. It doesn't work with the VN100 so it is excluded.
    // All other values seem to work fine.
    try {
      // Connect to sensor at it's default rate
      if (defaultBaudrate != 128000 && baudrate != 128000) {
        vs_.connect(port, defaultBaudrate);
        // Issues a change baudrate to the VectorNav sensor and then
        // reconnects the attached serial port at the new baudrate.
        vs_.changeBaudRate(baudrate);
        // Only makes it here once we have the default correct
        ROS_DEBUG_NAMED("vectornav", "Successfully connected with current baudrate: %d", vs_.baudrate());
        baudSet = true;
      }
    }
    // Catch all oddities
    catch (...) {
      // Disconnect if we had the wrong default and we were connected
      vs_.disconnect();
      ROS_WARN_NAMED("vectornav", "Failed to connect with baudrate: %d. Trying next baudrate...", defaultBaudrate);
      ros::Duration(0.2).sleep();
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
  if (vs_.verifySensorConnectivity()) {
    ROS_INFO_NAMED("vectornav", "Successfully connected: %s @ %d", vs_.port().c_str(), vs_.baudrate());
  } else {
    ROS_ERROR_NAMED("vectornav", "No device communication");
    ROS_WARN_NAMED("vectornav", "Please input a valid baud rate. Valid are:");
    ROS_WARN_NAMED("vectornav", "9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
    ROS_WARN_NAMED("vectornav", "With the test IMU 128000 did not work, all others worked fine.");
  }
  // Query the sensor's model number.
  std::string mn = vs_.readModelNumber();
  std::string fv = vs_.readFirmwareVersion();
  uint32_t hv = vs_.readHardwareRevision();
  uint32_t sn = vs_.readSerialNumber();
  ROS_INFO_NAMED("vectornav", "Model Number: %s, Firmware Version: %s", mn.c_str(), fv.c_str());
  ROS_INFO_NAMED("vectornav", "Hardware Revision : %d, Serial Number : %d", hv, sn);

  // Set the device info for passing to the packet callback function
  device_family_ = vs_.determineDeviceFamily();
}

void Vectornav::disconnect_device()
{
  // Node has been terminated
  vs_.unregisterAsyncPacketReceivedHandler();
  ros::Duration(0.1).sleep();
  ROS_INFO_NAMED("vectornav", "Unregisted the Packet Received Handler");
  vs_.disconnect();
  ros::Duration(0.1).sleep();
  ROS_INFO_NAMED("vectornav", "Device disconnected successfully");
}

void Vectornav::configure_device()
{
  int package_rate = 0;
  for (int allowed_rate : {1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200, 0}) {
    package_rate = allowed_rate;
    if ((package_rate % params_.async_output_rate) == 0 && (package_rate % params_.imu_output_rate) == 0) break;
  }
  ROS_ASSERT_MSG(
    package_rate != 0,
    "imu_output_rate (%d) or async_output_rate (%d) is not in 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, "
    "200 Hz",
    params_.imu_output_rate, params_.async_output_rate);
  imu_stride_ = static_cast<unsigned int>(package_rate / params_.imu_output_rate);
  output_stride_ = static_cast<unsigned int>(package_rate / params_.async_output_rate);
  ROS_INFO_NAMED("vectornav", "Package Receive Rate: %d Hz", package_rate);
  ROS_INFO_NAMED("vectornav", "General Publish Rate: %d Hz", params_.async_output_rate);
  ROS_INFO_NAMED("vectornav", "IMU Publish Rate: %d Hz", params_.imu_output_rate);

  // Arbitrary value that indicates the maximum expected time between consecutive IMU messages
  maximum_imu_timestamp_difference_ = (1 / static_cast<double>(params_.imu_output_rate)) * 10;

  // Make sure no generic async output is registered
  vs_.writeAsyncDataOutputType(vn::protocol::uart::AsciiAsync::VNOFF);

  using namespace vn::protocol::uart;
  // Configure binary output message
  vn::sensors::BinaryOutputRegister bor(
    ASYNCMODE_PORT1,
    params_.fixed_imu_rate / package_rate,  // update rate [ms]
    COMMONGROUP_QUATERNION | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE |
      COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES |
      (adjust_ros_timestamp_ ? COMMONGROUP_TIMESTARTUP : 0),
    TIMEGROUP_NONE | TIMEGROUP_GPSTOW | TIMEGROUP_GPSWEEK | TIMEGROUP_TIMEUTC, IMUGROUP_NONE,
    GPSGROUP_NONE,
    ATTITUDEGROUP_YPRU,  //<-- returning yaw pitch roll uncertainties
    INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELBODY | INSGROUP_ACCELECEF |
      INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU,
    GPSGROUP_NONE);

  // An empty output register for disabling output 2 and 3 if previously set
  vn::sensors::BinaryOutputRegister bor_none(
    0, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
    INSGROUP_NONE, GPSGROUP_NONE);

  vs_.writeBinaryOutput1(bor);
  vs_.writeBinaryOutput2(bor_none);
  vs_.writeBinaryOutput3(bor_none);

  // Setting reference frame
  vn::math::mat3f current_rotation_reference_frame { vs_.readReferenceFrameRotation() };
  ROS_INFO_STREAM_NAMED("vectornav", "Current rotation reference frame: " << current_rotation_reference_frame);

  if (has_rotation_reference_frame_ == true) {
    vn::math::mat3f const matrix_rotation_reference_frame {
      static_cast<float>(params_.rotation_reference_frame.at(0)),
      static_cast<float>(params_.rotation_reference_frame.at(1)),
      static_cast<float>(params_.rotation_reference_frame.at(2)),
      static_cast<float>(params_.rotation_reference_frame.at(3)),
      static_cast<float>(params_.rotation_reference_frame.at(4)),
      static_cast<float>(params_.rotation_reference_frame.at(5)),
      static_cast<float>(params_.rotation_reference_frame.at(6)),
      static_cast<float>(params_.rotation_reference_frame.at(7)),
      static_cast<float>(params_.rotation_reference_frame.at(8))
    };

    // Check diagonal to determine if the matrix is different, the rest of the values should be 0
    // There is no method to compare matrices directly
    if ( current_rotation_reference_frame.e00 != matrix_rotation_reference_frame.e00
      || current_rotation_reference_frame.e11 != matrix_rotation_reference_frame.e11
      || current_rotation_reference_frame.e22 != matrix_rotation_reference_frame.e22)
    {
      ROS_INFO_STREAM("Current rotation reference frame is different from the desired one: " << matrix_rotation_reference_frame);
      vs_.writeReferenceFrameRotation(matrix_rotation_reference_frame, true);
      current_rotation_reference_frame = vs_.readReferenceFrameRotation();
      ROS_INFO_STREAM("New rotation reference frame: " << current_rotation_reference_frame);
      ROS_INFO_STREAM("Restarting device to save new reference frame");
      vs_.writeSettings(true);
      vs_.reset();
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("vectornav", "Current rotation reference frame is the same as the desired one: " << matrix_rotation_reference_frame);
    }
  }

  // Register async callback function
  vs_.registerAsyncPacketReceivedHandler(static_cast<void*>(this), &Vectornav::binary_async_message_received);
}

void Vectornav::binary_async_message_received(void* aux, vn::protocol::uart::Packet & p, size_t index)
{
  static_cast<Vectornav*>(aux)->binary_async_message_received_(p, index);
}

void Vectornav::binary_async_message_received_(vn::protocol::uart::Packet & p, size_t index)
{
  // evaluate time first, to have it as close to the measurement time as possible
  const ros::Time ros_time = ros::Time::now();

  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  ros::Time time = get_timestamp(cd, ros_time);

    // Only publish if timestamp did not go back in time
  if (newest_timestamp_ < time)
  {
    newest_timestamp_ = time;
    // IMU
    std::unique_lock<std::mutex> lock(mtx_samples_);
    if (((pkg_count_ % imu_stride_) == 0 && pub_imu_.getNumSubscribers() > 0) || take_samples_) {
      sensor_msgs::Imu msgIMU;
      if (fill_imu_msg(cd, msgIMU, time) == true)
      {
        if ((pkg_count_ % imu_stride_) == 0)
          pub_imu_.publish(msgIMU);
        if (take_samples_)
          samples_.push_back({msgIMU.linear_acceleration.x, msgIMU.linear_acceleration.y, msgIMU.linear_acceleration.z});
      }
    }
    lock.unlock();

    if ((pkg_count_ % output_stride_) == 0) {
      // Magnetic Field
      if (pub_mag_.getNumSubscribers() > 0) {
        sensor_msgs::MagneticField mag_msg{};
        fill_mag_msg(cd, mag_msg, time);
        pub_mag_.publish(mag_msg);
      }

      // Temperature
      if (pub_temp_.getNumSubscribers() > 0) {
        sensor_msgs::Temperature temp_msg{};
        fill_temp_msg(cd, temp_msg, time);
        pub_temp_.publish(temp_msg);
      }

      // Barometer
      if (pub_pres_.getNumSubscribers() > 0) {
        sensor_msgs::FluidPressure pres_msg{};
        fill_pres_msg(cd, pres_msg, time);
        pub_pres_.publish(pres_msg);
      }

      // GPS
      if (
        device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100 &&
        pub_gps_.getNumSubscribers() > 0) {
        sensor_msgs::NavSatFix gps_msg{};
        fill_gps_msg(cd, gps_msg, time);
        pub_gps_.publish(gps_msg);
      }

      // Odometry
      if (
        device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100 &&
        pub_odom_.getNumSubscribers() > 0) {
        nav_msgs::Odometry msgOdom{};
        fill_odom_msg(cd, msgOdom, time);
        pub_odom_.publish(msgOdom);
      }

      // INS
      if (
        device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100 &&
        pub_ins_.getNumSubscribers() > 0) {
        vectornav::Ins ins_msg{};
        fill_ins_msg(cd, ins_msg, time);
        pub_ins_.publish(ins_msg);
      }
    }
  }
  else
  {
    ROS_WARN("IMU message filtered, timestamp went back in time");
  }
  ++pkg_count_;
}

ros::Time Vectornav::get_timestamp(vn::sensors::CompositeData & cd, const ros::Time & ros_time)
{
  if (!cd.hasTimeStartup() || !adjust_ros_timestamp_) {
    return (ros_time);  // don't adjust timestamp
  }

  const double sensor_time = cd.timeStartup() * 1e-9;  // time in seconds

  if (average_time_difference_ == 0) {       // first call
    ros_start_time_ = ros_time;
    average_time_difference_ = static_cast<double>(-sensor_time);
    last_sensor_time_ = sensor_time;
  }

  if (!validate_timestamp(sensor_time))
    return ros::Time(0);

  // difference between node startup and current ROS time
  const double ros_dt = (ros_time - ros_start_time_).toSec();

  // Do not use ros_dt to calculate average_time_difference if it is smaller than last measurement
  if (ros_dt > biggest_ros_dt_)
  {
    // difference between elapsed ROS time and time since sensor startup
    const double dt = ros_dt - sensor_time;
    // compute exponential moving average
    const double alpha = 0.001;  // average over rougly 1000 samples
    average_time_difference_ = average_time_difference_*(1.0-alpha) + alpha*dt;
    biggest_ros_dt_ = ros_dt;
  }
  else
  {
    ROS_WARN("WARNING: ros_dt: %f is smaller than biggest_ros_dt: %f."
      "This ros_dt will not be used to calculate average_time_difference", ros_dt, biggest_ros_dt_);
  }

  // adjust sensor time by average difference to ROS time
  const ros::Time adj_time { ros_start_time_ + ros::Duration(average_time_difference_ + sensor_time) };

  return adj_time;
}

bool Vectornav::validate_timestamp(double const sensor_time)
{
  bool isValid = true;

  // Do not calcuate timestamp if difference between current and previous timestamp is higher than expected
  if (std::abs(sensor_time - last_sensor_time_) > maximum_imu_timestamp_difference_)
  {
    ROS_WARN("WARNING: difference between sensor_time: %f and last_sensor_time: %f is bigger than "
      "maximum_imu_timestamp_difference: %f. Returning an invalid timestamp to reject "
       "this measurement", sensor_time, last_sensor_time_, maximum_imu_timestamp_difference_);
    isValid = false;
  }

  last_sensor_time_ = sensor_time;

  if (isValid)
  {
    // Do not calcuate timestamp nor update newest_sensor_time if sensor_time is smaller than last measurement
    if (sensor_time < newest_sensor_time_)
    {
      ROS_WARN("WARNING: sensor_time: %f is smaller than newest_sensor_time: %f."
        "Returning an invalid timestamp to reject this measurement", sensor_time, newest_sensor_time_);
      isValid = false;
    }
    else
    {
      newest_sensor_time_ = sensor_time;
    }
  }

  return isValid;
}

bool Vectornav::fill_imu_msg(vn::sensors::CompositeData & cd, sensor_msgs::Imu & imu_msg, const ros::Time & ros_time)
{
  imu_msg.header.stamp = ros_time;
  imu_msg.header.frame_id = params_.imu_frame_id;

  if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration()) {
    vn::math::vec4f q  { cd.quaternion() };
    vn::math::vec3f ar { cd.angularRate() };
    vn::math::vec3f al { cd.acceleration() };

    if (cd.hasAttitudeUncertainty()) {
      vn::math::vec3f orientationStdDev = cd.attitudeUncertainty();
      imu_msg.orientation_covariance[0] =
        pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Roll
      imu_msg.orientation_covariance[4] =
        pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians Pitch
      imu_msg.orientation_covariance[8] =
        pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Yaw
    }

    auto validate_quaternion = [](vn::math::vec4f const & q) {
      return std::isfinite(q[0]) and std::isfinite(q[1]) and std::isfinite(q[2]) and std::isfinite(q[3])
      && (std::abs(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] - 1.0f) < 0.01)
      && !(q[0] == 0. && q[1] == 0. && q[2] == 0. && q[3] == 0.);
    };

    auto validate_vector = [](vn::math::vec3f const & v) {
      return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };

    if (!validate_quaternion(q) || !validate_vector(ar) || !validate_vector(al))
    {
        invalid_data_++;
        ROS_WARN_THROTTLE(1, "Invalid data (%d until now). Orientation: %f, %f, %f, %f. Angular velocity: %f, %f, %f. Linear Acceleration: %f, %f, %f",
                    invalid_data_, q[0], q[1], q[2], q[3],
                    ar[0], ar[1], ar[2], al[0], al[1], al[2]);

        // Shutdown node if more than max_invalid_packets are received consecutively
        if ((max_invalid_packets_ != -1) && (invalid_data_ >= max_invalid_packets_))
          ros::shutdown();

        return false;
    }
    else
    {
      invalid_data_ = 0;
      //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
      if (params_.tf_ned_to_enu) {
        // If we want the orientation to be based on the reference label on the imu
        tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
        geometry_msgs::Quaternion quat_msg;

        if (params_.frame_based_enu) {
          // Create a rotation from NED -> ENU
          tf2::Quaternion q_rotate;
          q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
          // Apply the NED to ENU rotation such that the coordinate frame matches
          tf2_quat = q_rotate * tf2_quat;
          quat_msg = tf2::toMsg(tf2_quat);

          // Since everything is in the normal frame, no flipping required
          imu_msg.angular_velocity.x = ar[0];
          imu_msg.angular_velocity.y = ar[1];
          imu_msg.angular_velocity.z = ar[2];

          imu_msg.linear_acceleration.x = al[0];
          imu_msg.linear_acceleration.y = al[1];
          imu_msg.linear_acceleration.z = al[2];
        } else {
          // put into ENU - swap X/Y, invert Z
          quat_msg.x = q[1];
          quat_msg.y = q[0];
          quat_msg.z = -q[2];
          quat_msg.w = q[3];

          // Flip x and y then invert z
          imu_msg.angular_velocity.x = ar[1];
          imu_msg.angular_velocity.y = ar[0];
          imu_msg.angular_velocity.z = -ar[2];
          // Flip x and y then invert z
          imu_msg.linear_acceleration.x = al[1];
          imu_msg.linear_acceleration.y = al[0];
          imu_msg.linear_acceleration.z = -al[2];

          if (cd.hasAttitudeUncertainty()) {
            vn::math::vec3f orientationStdDev { cd.attitudeUncertainty() };
            imu_msg.orientation_covariance[0] =
              pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians pitch
            imu_msg.orientation_covariance[4] =
              pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians Roll
            imu_msg.orientation_covariance[8] =
              pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Yaw
          }
        }

        imu_msg.orientation = quat_msg;
      }
      else if (params_.tf_ned_to_nwu) {
        tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
        geometry_msgs::Quaternion quat_msg;

        if (params_.frame_based_nwu) {
          // Create a rotation from NED -> NWU
          tf2::Quaternion q_rotate;
          q_rotate.setRPY(M_PI, 0.0, 0.0);
          // Apply the NED to ENU rotation such that the coordinate frame matches
          tf2_quat = q_rotate * tf2_quat;
          quat_msg = tf2::toMsg(tf2_quat);

          // Since everything is in the normal frame, no flipping required
          imu_msg.angular_velocity.x = ar[0];
          imu_msg.angular_velocity.y = ar[1];
          imu_msg.angular_velocity.z = ar[2];

          imu_msg.linear_acceleration.x = al[0];
          imu_msg.linear_acceleration.y = al[1];
          imu_msg.linear_acceleration.z = al[2];
        } else {
          // put into NWU
          quat_msg.x = q[0];
          quat_msg.y = -q[1];
          quat_msg.z = -q[2];
          quat_msg.w = q[3];

          // Invert y and z
          imu_msg.angular_velocity.x = ar[0];
          imu_msg.angular_velocity.y = -ar[1];
          imu_msg.angular_velocity.z = -ar[2];
          // Invert y and z
          imu_msg.linear_acceleration.x = al[0];
          imu_msg.linear_acceleration.y = -al[1];
          imu_msg.linear_acceleration.z = -al[2];

          if (cd.hasAttitudeUncertainty()) {
            vn::math::vec3f orientationStdDev { cd.attitudeUncertainty() };
            imu_msg.orientation_covariance[0] =
              pow(orientationStdDev[0] * M_PI / 180, 2);  // Convert to radians roll
            imu_msg.orientation_covariance[4] =
              pow(orientationStdDev[1] * M_PI / 180, 2);  // Convert to radians pitch
            imu_msg.orientation_covariance[8] =
              pow(orientationStdDev[2] * M_PI / 180, 2);  // Convert to radians Yaw
          }
        }
        imu_msg.orientation = quat_msg;
      } else {
        imu_msg.orientation.x = q[0];
        imu_msg.orientation.y = q[1];
        imu_msg.orientation.z = q[2];
        imu_msg.orientation.w = q[3];

        imu_msg.angular_velocity.x = ar[0];
        imu_msg.angular_velocity.y = ar[1];
        imu_msg.angular_velocity.z = ar[2];
        imu_msg.linear_acceleration.x = al[0];
        imu_msg.linear_acceleration.y = al[1];
        imu_msg.linear_acceleration.z = al[2];
      }
      // Covariances pulled from parameters
      std::copy(params_.angular_vel_covariance.begin(), params_.angular_vel_covariance.end(),
                imu_msg.angular_velocity_covariance.begin());
      std::copy(params_.linear_accel_covariance.begin(), params_.linear_accel_covariance.end(),
                imu_msg.linear_acceleration_covariance.begin());
    }
    return true;
  }
  ROS_WARN("IMU invalid data, discarding message");
  return false;
}

bool Vectornav::fill_mag_msg(vn::sensors::CompositeData & cd, sensor_msgs::MagneticField & mag_msg, const ros::Time & ros_time)
{
  mag_msg.header.stamp = ros_time;
  mag_msg.header.frame_id = params_.imu_frame_id;

  // Magnetic Field
  if (cd.hasMagnetic()) {
    auto const mag { cd.magnetic() };
    mag_msg.magnetic_field.x = mag[0];
    mag_msg.magnetic_field.y = mag[1];
    mag_msg.magnetic_field.z = mag[2];
  }
}

bool Vectornav::fill_temp_msg(vn::sensors::CompositeData & cd, sensor_msgs::Temperature & temp_msg, const ros::Time & ros_time)
{
  temp_msg.header.stamp = ros_time;
  temp_msg.header.frame_id = params_.imu_frame_id;
  if (cd.hasTemperature()) {
    float const temp { cd.temperature() };
    temp_msg.temperature = static_cast<double>(temp);
  }
}

bool Vectornav::fill_pres_msg(vn::sensors::CompositeData & cd, sensor_msgs::FluidPressure & pres_msg, const ros::Time & ros_time)
{
  pres_msg.header.stamp = ros_time;
  pres_msg.header.frame_id = params_.imu_frame_id;
  if (cd.hasPressure()) {
    float const pres { cd.pressure() };
    pres_msg.fluid_pressure = static_cast<double>(pres);
  }}

bool Vectornav::fill_gps_msg(vn::sensors::CompositeData & cd, sensor_msgs::NavSatFix & gps_msg, const ros::Time & ros_time)
{
  // Check with vectornav different of VN-100
  gps_msg.header.stamp = ros_time;
  gps_msg.header.frame_id = params_.imu_frame_id;

  if (cd.hasPositionEstimatedLla()) {
    auto const lla { cd.positionEstimatedLla() };

    gps_msg.latitude = lla[0];
    gps_msg.longitude = lla[1];
    gps_msg.altitude = lla[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      gps_msg.position_covariance[0] = posVariance;  // East position variance
      gps_msg.position_covariance[4] = posVariance;  // North position vaciance
      gps_msg.position_covariance[8] = posVariance;  // Up position variance

      // mark gps fix as not available if the outputted standard deviation is 0
      if (cd.positionUncertaintyEstimated() != 0.0) {
        // Position available
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      } else {
        // position not detected
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      }

      // add the type of covariance to the gps message
      gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
  }
}

bool Vectornav::fill_odom_msg(vn::sensors::CompositeData & cd, nav_msgs::Odometry & odom_msg, const ros::Time & ros_time)
{
  // Check with vectornav different of VN-100
  odom_msg.header.stamp = ros_time;
  odom_msg.child_frame_id = params_.imu_frame_id;
  odom_msg.header.frame_id = params_.map_frame_id;

  if (cd.hasPositionEstimatedEcef()) {
    // add position as earth fixed frame
    vn::math::vec3d pos { cd.positionEstimatedEcef() };

    if (!initial_position_set_) {
      ROS_INFO_NAMED("vectornav", "Set initial position to %f %f %f", pos[0], pos[1], pos[2]);
      initial_position_set_ = true;
      initial_position_.x = pos[0];
      initial_position_.y = pos[1];
      initial_position_.z = pos[2];
    }

    odom_msg.pose.pose.position.x = pos[0] - initial_position_[0];
    odom_msg.pose.pose.position.y = pos[1] - initial_position_[1];
    odom_msg.pose.pose.position.z = pos[2] - initial_position_[2];

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasPositionUncertaintyEstimated()) {
      double posVariance = pow(cd.positionUncertaintyEstimated(), 2);
      odom_msg.pose.covariance[0] = posVariance;   // x-axis position variance
      odom_msg.pose.covariance[7] = posVariance;   // y-axis position vaciance
      odom_msg.pose.covariance[14] = posVariance;  // z-axis position variance
    }
  }

  if (cd.hasQuaternion()) {
    vn::math::vec4f q { cd.quaternion() };

    if (!params_.tf_ned_to_enu && !params_.tf_ned_to_nwu) {
      // output in NED frame
      odom_msg.pose.pose.orientation.x = q[0];
      odom_msg.pose.pose.orientation.y = q[1];
      odom_msg.pose.pose.orientation.z = q[2];
      odom_msg.pose.pose.orientation.w = q[3];
    } else if (params_.tf_ned_to_enu && params_.frame_based_enu) {
      // standard conversion from NED to ENU frame
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      // Create a rotation from NED -> ENU
      tf2::Quaternion q_rotate;
      q_rotate.setRPY(M_PI, 0.0, M_PI / 2);
      // Apply the NED to ENU rotation such that the coordinate frame matches
      tf2_quat = q_rotate * tf2_quat;
      odom_msg.pose.pose.orientation = tf2::toMsg(tf2_quat);
    } else if (params_.tf_ned_to_enu && !params_.frame_based_enu) {
      // alternative method for conversion to ENU frame (leads to another result)
      // put into ENU - swap X/Y, invert Z
      odom_msg.pose.pose.orientation.x = q[1];
      odom_msg.pose.pose.orientation.y = q[0];
      odom_msg.pose.pose.orientation.z = -q[2];
      odom_msg.pose.pose.orientation.w = q[3];
    } else if (params_.tf_ned_to_nwu && params_.frame_based_nwu) {
      // standard conversion from NED to ENU frame
      tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
      // Create a rotation from NED -> ENU
      tf2::Quaternion q_rotate;
      q_rotate.setRPY(M_PI, 0.0, 0.0);
      // Apply the NED to ENU rotation such that the coordinate frame matches
      tf2_quat = q_rotate * tf2_quat;
      odom_msg.pose.pose.orientation = tf2::toMsg(tf2_quat);
    } else if (params_.tf_ned_to_nwu && !params_.frame_based_nwu) {
      // alternative method for conversion to ENU frame (leads to another result)
      // put into ENU - swap X/Y, invert Z
      odom_msg.pose.pose.orientation.x = q[0];
      odom_msg.pose.pose.orientation.y = -q[1];
      odom_msg.pose.pose.orientation.z = -q[2];
      odom_msg.pose.pose.orientation.w = q[3];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasAttitudeUncertainty()) {
      vn::math::vec3f orientationStdDev { cd.attitudeUncertainty() };
      // convert the standard deviation values from all three axis from degrees to radiant and calculate the variances from these (squared), which are assigned to the covariance matrix.
      if ( (!params_.tf_ned_to_enu && !params_.tf_ned_to_nwu) || params_.frame_based_enu || params_.frame_based_nwu) {
        // standard assignment of variance values for NED frame and conversion to ENU frame by rotation
        odom_msg.pose.covariance[21] = pow(orientationStdDev[0] * M_PI / 180, 2);  // roll variance
        odom_msg.pose.covariance[28] = pow(orientationStdDev[1] * M_PI / 180, 2);  // pitch variance
        odom_msg.pose.covariance[35] = pow(orientationStdDev[2] * M_PI / 180, 2);  // yaw variance
      } else if (params_.tf_ned_to_enu && !params_.frame_based_enu){
        // variance assignment for conversion by swapping and inverting (not frame_based_enu or frame_based_nwu)
        // TODO not supported yet
      } else if (params_.tf_ned_to_nwu && !params_.frame_based_nwu) {
        // variance assignment for conversion by swapping and inverting (not frame_based_enu or frame_based_nwu)
        // TODO not supported yet
      }
    }
  }

  // Add the velocity in the body frame (frame_id) to the message
  if (cd.hasVelocityEstimatedBody()) {
    vn::math::vec3f vel { cd.velocityEstimatedBody() };

    if ( (!params_.tf_ned_to_enu && !params_.tf_ned_to_nwu) || params_.frame_based_enu || params_.frame_based_nwu) {
      // standard assignment of values for NED frame and conversion to ENU or NWU frame by rotation
      odom_msg.twist.twist.linear.x = vel[0];
      odom_msg.twist.twist.linear.y = vel[1];
      odom_msg.twist.twist.linear.z = vel[2];
    } else if (params_.tf_ned_to_enu && !params_.frame_based_enu) {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      odom_msg.twist.twist.linear.x = vel[1];
      odom_msg.twist.twist.linear.y = vel[0];
      odom_msg.twist.twist.linear.z = -vel[2];
    } else if (params_.tf_ned_to_nwu && !params_.frame_based_nwu) {
      // value assignment for conversion by swapping and inverting (not frame_based_nwu)
      // Invert y and z
      odom_msg.twist.twist.linear.x = vel[0];
      odom_msg.twist.twist.linear.y = -vel[1];
      odom_msg.twist.twist.linear.z = -vel[2];
    }

    // Read the estimation uncertainty (1 Sigma) from the sensor and write it to the covariance matrix.
    if (cd.hasVelocityUncertaintyEstimated()) {
      double velVariance = pow(cd.velocityUncertaintyEstimated(), 2);
      odom_msg.twist.covariance[0] = velVariance;   // x-axis velocity variance
      odom_msg.twist.covariance[7] = velVariance;   // y-axis velocity vaciance
      odom_msg.twist.covariance[14] = velVariance;  // z-axis velocity variance

      // set velocity variances to a high value if no data is available (this is the case at startup during INS is initializing)
      if (
        odom_msg.twist.twist.linear.x == 0 && odom_msg.twist.twist.linear.y == 0 &&
        odom_msg.twist.twist.linear.z == 0 && odom_msg.twist.covariance[0] == 0 &&
        odom_msg.twist.covariance[7] == 0 && odom_msg.twist.covariance[14] == 0) {
        odom_msg.twist.covariance[0] = 200;
        odom_msg.twist.covariance[7] = 200;
        odom_msg.twist.covariance[15] = 200;
      }
    }
  }

  if (cd.hasAngularRate()) {
    vn::math::vec3f ar { cd.angularRate() };

    if ( (!params_.tf_ned_to_enu && !params_.tf_ned_to_nwu) || params_.frame_based_enu || params_.frame_based_nwu) {
      // standard assignment of values for NED frame and conversion to ENU or NWU frame by rotation
      odom_msg.twist.twist.angular.x = ar[0];
      odom_msg.twist.twist.angular.y = ar[1];
      odom_msg.twist.twist.angular.z = ar[2];
    } else if (params_.tf_ned_to_enu && !params_.frame_based_enu) {
      // value assignment for conversion by swapping and inverting (not frame_based_enu)
      // Flip x and y then invert z
      odom_msg.twist.twist.angular.x = ar[1];
      odom_msg.twist.twist.angular.y = ar[0];
      odom_msg.twist.twist.angular.z = -ar[2];
    } else if (params_.tf_ned_to_nwu && !params_.frame_based_nwu) {
      // value assignment for conversion by swapping and inverting (not frame_based_nwu)
      // Invert y and z
      odom_msg.twist.twist.angular.x = ar[0];
      odom_msg.twist.twist.angular.y = -ar[1];
      odom_msg.twist.twist.angular.z = -ar[2];
    }

    // add covariance matrix of the measured angular rate to odom message.
    // go through matrix rows
    for (int row = 0; row < 3; row++) {
      // go through matrix columns
      for (int col = 0; col < 3; col++) {
        // Target matrix has 6 rows and 6 columns, source matrix has 3 rows and 3 columns. The covariance values are put into the fields (3, 3) to (5, 5) of the destination matrix.
        odom_msg.twist.covariance[(row + 3) * 6 + (col + 3)] =
          params_.angular_vel_covariance[row * 3 + col];
      }
    }
  }

  return true;
}

bool Vectornav::fill_ins_msg(vn::sensors::CompositeData & cd, vectornav::Ins & ins_msg, const ros::Time & ros_time)
{
  // Check with vectornav different of VN-100
  ins_msg.header.stamp = ros_time;
  ins_msg.header.frame_id = params_.imu_frame_id;

  if (cd.hasInsStatus()) {
    auto const insStatus { cd.insStatus() };
    ins_msg.insStatus = static_cast<uint16_t>(insStatus);
  }

  if (cd.hasTow()) {
    ins_msg.time = cd.tow();
  }

  if (cd.hasWeek()) {
    ins_msg.week = cd.week();
  }

  if (cd.hasTimeUtc()) {
    auto const utcTime { cd.timeUtc() };
    char const * utcTimeBytes = reinterpret_cast<char const *>(&utcTime);
    //ins_msg.utcTime bytes are in Little Endian Byte Order
    std::memcpy(&ins_msg.utcTime, utcTimeBytes, 8);
  }

  if (cd.hasYawPitchRoll()) {
    vn::math::vec3f const rpy { cd.yawPitchRoll() };
    ins_msg.yaw = rpy[0];
    ins_msg.pitch = rpy[1];
    ins_msg.roll = rpy[2];
  }

  if (cd.hasPositionEstimatedLla()) {
    vn::math::vec3d const lla { cd.positionEstimatedLla() };
    ins_msg.latitude = lla[0];
    ins_msg.longitude = lla[1];
    ins_msg.altitude = lla[2];
  }

  if (cd.hasVelocityEstimatedNed()) {
    vn::math::vec3f const nedVel { cd.velocityEstimatedNed() };
    ins_msg.nedVelX = nedVel[0];
    ins_msg.nedVelY = nedVel[1];
    ins_msg.nedVelZ = nedVel[2];
  }

  if (cd.hasAttitudeUncertainty()) {
    vn::math::vec3f const attUncertainty { cd.attitudeUncertainty() };
    ins_msg.attUncertainty[0] = attUncertainty[0];
    ins_msg.attUncertainty[1] = attUncertainty[1];
    ins_msg.attUncertainty[2] = attUncertainty[2];
  }

  if (cd.hasPositionUncertaintyEstimated()) {
    ins_msg.posUncertainty = cd.positionUncertaintyEstimated();
  }

  if (cd.hasVelocityUncertaintyEstimated()) {
    ins_msg.velUncertainty = cd.velocityUncertaintyEstimated();
  }}

}  // namespace vectornav
