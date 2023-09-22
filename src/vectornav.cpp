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

  auto logger = rclcpp::get_logger("serial_optimizer");

  RCLCPP_INFO(logger, "Open port for optimization");
  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    RCLCPP_WARN(logger, "Can't open port for optimization");
    return false;
  }

  RCLCPP_DEBUG(logger, "Setting port to ASYNCY_LOW_LATENCY");
  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  RCLCPP_DEBUG(logger, "Closing port");
  ::close(portFd);
  return true;
}
#elif
bool optimize_serial_communication(str::string portName) {
  RCLCPP_DEBUG(logger, 
                 "Serial port low latency optimization not implemented on build platform");
  return true;
}
#endif

namespace vectornav
{

Vectornav::Vectornav()
: Node("vectornav")
, vs_{}
{
  RCLCPP_INFO(get_logger(), "Initializing vectornav device");

  // Read parameters from the parameter server
  RCLCPP_DEBUG(get_logger(), "Reading parameters from server");
  read_parameters();

  // Optimize serial low latencty communication
  RCLCPP_DEBUG(get_logger(), "Optimize serial low latencty communication");
  optimize_serial_communication(params_.port);

  // Connect to the device
  RCLCPP_INFO(get_logger(), "Connecting to: %s @ %d", params_.port.c_str(), params_.baudrate);
  connect_device(params_.port, params_.baudrate);

  RCLCPP_DEBUG(get_logger(), "Advertising topics");
  advertise_topics();

  RCLCPP_DEBUG(get_logger(), "Advertising services");
  advertise_services();

  RCLCPP_DEBUG(get_logger(), "Configuring device");
  configure_device();

  RCLCPP_DEBUG(get_logger(), "Finished initialization");
}

Vectornav::~Vectornav()
{
  // Node has been terminated
  vs_.unregisterAsyncPacketReceivedHandler();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  RCLCPP_INFO(get_logger(), "Unregisted the Packet Received Handler");
  vs_.disconnect();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  RCLCPP_INFO(get_logger(), "Device disconnected successfully"); 
}

void Vectornav::read_parameters()
{
  declare_parameter<std::string>("map_frame_id", params_.map_frame_id);
  declare_parameter<std::string>("frame_id", params_.imu_frame_id);
  declare_parameter<bool>("tf_ned_to_enu", params_.tf_ned_to_enu);
  declare_parameter<bool>("frame_based_enu", params_.frame_based_enu);
  declare_parameter<bool>("tf_ned_to_nwu", params_.tf_ned_to_nwu);
  declare_parameter<bool>("frame_based_nwu", params_.frame_based_nwu);
  declare_parameter<bool>("adjust_ros_timestamp", params_.adjust_ros_timestamp);
  declare_parameter<int>("async_output_rate", params_.async_output_rate);
  declare_parameter<int>("imu_output_rate", params_.imu_output_rate);
  declare_parameter<std::string>("serial_port", params_.port);
  declare_parameter<int>("serial_baud", params_.baudrate);
  declare_parameter<int>("fixed_imu_rate", params_.fixed_imu_rate);
  declare_parameter<int>("max_invalid_packets", params_.max_invalid_packets);
  declare_parameter<bool>("acc_bias_enable", params_.acc_bias_enable);
  declare_parameter<double>("set_acc_bias_seconds", params_.set_acc_bias_seconds);

  declare_parameter<std::vector<double>>("linear_accel_covariance", params_.linear_accel_covariance);
  declare_parameter<std::vector<double>>("angular_vel_covariance", params_.angular_vel_covariance);
  declare_parameter<std::vector<double>>("orientation_covariance", params_.orientation_covariance);
  declare_parameter<bool>("has_rotation_reference_frame", params_.has_rotation_reference_frame);
  declare_parameter<std::vector<double>>("rotation_reference_frame", params_.rotation_reference_frame);


  get_parameter("map_frame_id", params_.map_frame_id);
  get_parameter("frame_id", params_.imu_frame_id);
  get_parameter("tf_ned_to_enu", params_.tf_ned_to_enu);
  get_parameter("frame_based_enu", params_.frame_based_enu);
  get_parameter("tf_ned_to_nwu", params_.tf_ned_to_nwu);
  get_parameter("frame_based_nwu", params_.frame_based_nwu);
  get_parameter("adjust_ros_timestamp", params_.adjust_ros_timestamp);
  get_parameter("async_output_rate", params_.async_output_rate);
  get_parameter("imu_output_rate", params_.imu_output_rate);
  get_parameter("serial_port", params_.port);
  get_parameter("serial_baud", params_.baudrate);
  get_parameter("fixed_imu_rate", params_.fixed_imu_rate);
  get_parameter("max_invalid_packets", params_.max_invalid_packets);
  get_parameter("acc_bias_enable", params_.acc_bias_enable);
  get_parameter("set_acc_bias_seconds", params_.set_acc_bias_seconds);

  get_parameter("linear_accel_covariance", params_.linear_accel_covariance);
  get_parameter("angular_vel_covariance", params_.angular_vel_covariance);
  get_parameter("orientation_covariance", params_.orientation_covariance); // Is it used?
  get_parameter("has_rotation_reference_frame", params_.has_rotation_reference_frame);
  get_parameter("rotation_reference_frame", params_.rotation_reference_frame);
}

void Vectornav::advertise_topics()
{
  pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("~/data", rclcpp::SensorDataQoS());
  pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>("~/mag", rclcpp::SensorDataQoS());
  pub_temp_ = create_publisher<sensor_msgs::msg::Temperature>("~/temperature", rclcpp::SensorDataQoS());
  pub_pres_ = create_publisher<sensor_msgs::msg::FluidPressure>("~/atm_pressure", rclcpp::SensorDataQoS());

  // Filter topics (not supported by VN-100)
  if (device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100) {
    pub_gps_ = create_publisher<sensor_msgs::msg::NavSatFix>("~/global_position/raw/fix", rclcpp::SensorDataQoS());
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SensorDataQoS());
    pub_ins_ = create_publisher<vectornav::msg::Ins>("~/INS", rclcpp::SensorDataQoS());
  }
}

void Vectornav::advertise_services()
{
  srv_set_acc_bias_ = create_service<vectornav::srv::SetFrameHorizontal>(
    "set_acc_bias", std::bind(&Vectornav::set_acc_bias, this, std::placeholders::_1, std::placeholders::_2));
  // Filter unnecessary services (not supported by VN-100)
  if (device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100) {
    srv_reset_odom_ = create_service<std_srvs::srv::Empty>(
      "reset_odom", std::bind(&Vectornav::reset_odometry, this, std::placeholders::_1, std::placeholders::_2));
  }
}

void Vectornav::set_acc_bias(
    const std::shared_ptr<vectornav::srv::SetFrameHorizontal::Request> req,
    std::shared_ptr<vectornav::srv::SetFrameHorizontal::Response> resp)
{
  RCLCPP_INFO(get_logger(), "Set horizontal callback received. Setting horizontal frame.");
  vn::math::mat3f const gain {1., 0., 0.,
                              0., 1., 0.,
                              0., 0., 1.};
  
  if (req->reset) {
    vs_.writeAccelerationCompensation(gain, {0., 0., 0.}, true);
    vs_.writeSettings(true);

    resp->success = true;
    return;
  }

  std::unique_lock<std::mutex> lock(mtx_samples_);
    samples_.clear();
    samples_.reserve(static_cast<size_t>(req->duration * params_.fixed_imu_rate * 1.5));
    take_samples_ = true;
    auto const start = now();
  lock.unlock();

  rclcpp::Duration(std::chrono::microseconds(static_cast<uint64_t>(req->duration*1e6)));
  
  lock.lock();
    auto const end = now();
    take_samples_ = false;
    
    resp->samples_taken = samples_.size();
    resp->elapsed_time = (end - start).seconds();

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
  lock.unlock();

  auto const curr { vs_.readAccelerationCompensation() };

  vn::math::vec3f const bias {curr.b.x+static_cast<float>(bias_x), 
                              curr.b.y-static_cast<float>(bias_y), 
                              curr.b.z-static_cast<float>(bias_z-9.80665)};

  resp->bias_x = static_cast<double>(curr.b.x) - bias_x;
  resp->bias_y = static_cast<double>(curr.b.y) - bias_y;
  resp->bias_z = static_cast<double>(curr.b.z) - (bias_z-9.80665);

  resp->covariance_x = covariance_x;
  resp->covariance_y = covariance_y;
  resp->covariance_z = covariance_z;
  
  if (samples_.size() < 10) {
    RCLCPP_ERROR(get_logger(), "Not enough samples taken. Aborting.");
    resp->success = false;
  } else {
    resp->success = true;
    vs_.writeAccelerationCompensation(gain, {bias.x, bias.y, bias.z}, true);
    vs_.writeSettings(true);
  }
}

void Vectornav::reset_odometry(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> resp)
{
  RCLCPP_INFO(get_logger(), "Reset odometry callback received. Resetting odometry.");
  initial_position_set_ = false;
  return;
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
    RCLCPP_DEBUG(get_logger(), "Trying to connect with baudrate: %d", defaultBaudrate);
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
        RCLCPP_DEBUG(get_logger(), "Successfully connected with current baudrate: %d", vs_.baudrate());
        baudSet = true;
      }
    }
    // Catch all oddities
    catch (...) {
      // Disconnect if we had the wrong default and we were connected
      vs_.disconnect();
      RCLCPP_WARN(get_logger(), "Failed to connect with baudrate: %d. Trying next baudrate...", defaultBaudrate);
      rclcpp::Duration(std::chrono::microseconds(static_cast<uint64_t>(0.2*1e6)));
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
    RCLCPP_INFO(get_logger(), "Successfully connected: %s @ %d", vs_.port().c_str(), vs_.baudrate());
  } else {
    RCLCPP_ERROR(get_logger(), "No device communication");
    RCLCPP_WARN(get_logger(), "Please input a valid baud rate. Valid are:");
    RCLCPP_WARN(get_logger(), "9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600");
    RCLCPP_WARN(get_logger(), "With the test IMU 128000 did not work, all others worked fine.");
  }
  // Query the sensor's model number.
  std::string mn = vs_.readModelNumber();
  std::string fv = vs_.readFirmwareVersion();
  uint32_t hv = vs_.readHardwareRevision();
  uint32_t sn = vs_.readSerialNumber();
  RCLCPP_INFO(get_logger(), "Model Number: %s, Firmware Version: %s", mn.c_str(), fv.c_str());
  RCLCPP_INFO(get_logger(), "Hardware Revision : %d, Serial Number : %d", hv, sn);

  // Set the device info for passing to the packet callback function
  device_family_ = vs_.determineDeviceFamily();
}

void Vectornav::configure_device()
{
  int package_rate = 0;
  for (int allowed_rate : {1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200, 0}) {
    package_rate = allowed_rate;
    if ((package_rate % params_.async_output_rate) == 0 && (package_rate % params_.imu_output_rate) == 0) break;
  }
  if (package_rate == 0)
    RCLCPP_ERROR(get_logger(), 
      "imu_output_rate (%d) or async_output_rate (%d) is not in 1, 2,"
      " 4, 5, 10, 20, 25, 40, 50, 100, 200 Hz",
      params_.imu_output_rate, params_.async_output_rate);
  imu_stride_ = static_cast<unsigned int>(package_rate / params_.imu_output_rate);
  output_stride_ = static_cast<unsigned int>(package_rate / params_.async_output_rate);
  RCLCPP_INFO(get_logger(), "Package Receive Rate: %d Hz", package_rate);
  RCLCPP_INFO(get_logger(), "General Publish Rate: %d Hz", params_.async_output_rate);
  RCLCPP_INFO(get_logger(), "IMU Publish Rate: %d Hz", params_.imu_output_rate);

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
      (params_.adjust_ros_timestamp ? COMMONGROUP_TIMESTARTUP : 0),
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
  RCLCPP_INFO_STREAM(get_logger(), "Current rotation reference frame: " << current_rotation_reference_frame);

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
      RCLCPP_INFO_STREAM(get_logger(), "Current rotation reference frame is different from the desired one: " << matrix_rotation_reference_frame);
      vs_.writeReferenceFrameRotation(matrix_rotation_reference_frame, true);
      current_rotation_reference_frame = vs_.readReferenceFrameRotation();
      RCLCPP_INFO_STREAM(get_logger(), "New rotation reference frame: " << current_rotation_reference_frame);
      RCLCPP_INFO_STREAM(get_logger(), "Restarting device to save new reference frame");
      vs_.writeSettings(true);
      vs_.reset();
    }
    else
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "Current rotation reference frame is the same as the desired one: " << matrix_rotation_reference_frame);
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
  rclcpp::Time const ros_time = now();

  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
  rclcpp::Time const time = get_timestamp(cd, ros_time);

    // Only publish if timestamp did not go back in time
  if (newest_timestamp_.seconds() < time.seconds())
  {
    newest_timestamp_ = time;
    // IMU
    std::unique_lock<std::mutex> lock(mtx_samples_);
    if (((pkg_count_ % imu_stride_) == 0 && pub_imu_->get_subscription_count() > 0) || take_samples_) {
      sensor_msgs::msg::Imu msgIMU;
      if (fill_imu_msg(cd, msgIMU, time) == true)
      {
        if ((pkg_count_ % imu_stride_) == 0)
          pub_imu_->publish(msgIMU);
        if (take_samples_)
          samples_.push_back({msgIMU.linear_acceleration.x, msgIMU.linear_acceleration.y, msgIMU.linear_acceleration.z});
      }
    }
    lock.unlock();

    if ((pkg_count_ % output_stride_) == 0) {
      // Magnetic Field
      if (pub_mag_->get_subscription_count() > 0) {
        sensor_msgs::msg::MagneticField mag_msg{};
        fill_mag_msg(cd, mag_msg, time);
        pub_mag_->publish(mag_msg);
      }

      // Temperature
      if (pub_temp_->get_subscription_count() > 0) {
        sensor_msgs::msg::Temperature temp_msg{};
        fill_temp_msg(cd, temp_msg, time);
        pub_temp_->publish(temp_msg);
      }

      // Barometer
      if (pub_pres_->get_subscription_count() > 0) {
        sensor_msgs::msg::FluidPressure pres_msg{};
        fill_pres_msg(cd, pres_msg, time);
        pub_pres_->publish(pres_msg);
      }

      // GPS
      if (
        device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100 &&
        pub_gps_->get_subscription_count() > 0) {
        sensor_msgs::msg::NavSatFix gps_msg{};
        fill_gps_msg(cd, gps_msg, time);
        pub_gps_->publish(gps_msg);
      }

      // Odometry
      if (
        device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100 &&
        pub_odom_->get_subscription_count() > 0) {
        nav_msgs::msg::Odometry msgOdom{};
        fill_odom_msg(cd, msgOdom, time);
        pub_odom_->publish(msgOdom);
      }

      // INS
      if (
        device_family_ != vn::sensors::VnSensor::Family::VnSensor_Family_Vn100 &&
        pub_ins_->get_subscription_count() > 0) {
        vectornav::msg::Ins ins_msg{};
        fill_ins_msg(cd, ins_msg, time);
        pub_ins_->publish(ins_msg);
      }
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "IMU message filtered, timestamp went back in time");
  }
  ++pkg_count_;
}

rclcpp::Time Vectornav::get_timestamp(vn::sensors::CompositeData & cd, const rclcpp::Time & ros_time)
{
  if (!cd.hasTimeStartup() || !params_.adjust_ros_timestamp) {
    return (ros_time);  // don't adjust timestamp
  }

  const double sensor_time = cd.timeStartup() * 1e-9;  // time in seconds

  if (average_time_difference_ == 0) {       // first call
    ros_start_time_ = ros_time;
    average_time_difference_ = static_cast<double>(-sensor_time);
    last_sensor_time_ = sensor_time;
  }

  if (!validate_timestamp(sensor_time))
    return rclcpp::Time(0);

  // difference between node startup and current ROS time
  const double ros_dt = (ros_time - ros_start_time_).seconds();

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
    RCLCPP_WARN(get_logger(), "WARNING: ros_dt: %f is smaller than biggest_ros_dt: %f."
      "This ros_dt will not be used to calculate average_time_difference", ros_dt, biggest_ros_dt_);
  }

  // adjust sensor time by average difference to ROS time
  const rclcpp::Time adj_time { 
    ros_start_time_ + rclcpp::Duration(std::chrono::microseconds(static_cast<uint64_t>((average_time_difference_ + sensor_time)*1e6)))
  };

  return adj_time;
}

bool Vectornav::validate_timestamp(double const sensor_time)
{
  bool isValid = true;

  // Do not calcuate timestamp if difference between current and previous timestamp is higher than expected
  if (std::abs(sensor_time - last_sensor_time_) > maximum_imu_timestamp_difference_)
  {
    RCLCPP_WARN(get_logger(), "WARNING: difference between sensor_time: %f and last_sensor_time: %f is bigger than "
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
      RCLCPP_WARN(get_logger(), "WARNING: sensor_time: %f is smaller than newest_sensor_time: %f."
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

bool Vectornav::fill_imu_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::Imu & imu_msg, const rclcpp::Time & ros_time)
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
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
          "Invalid data (%d until now). Orientation: %f, %f, %f, %f. "
                                       "Angular velocity: %f, %f, %f. "
                                       "Linear Acceleration: %f, %f, %f",
          invalid_data_, q[0], q[1], q[2], q[3],
          ar[0], ar[1], ar[2], al[0], al[1], al[2]);

        // Shutdown node if more than max_invalid_packets are received consecutively
        if ((max_invalid_packets_ != -1) && (invalid_data_ >= max_invalid_packets_))
          rclcpp::shutdown();

        return false;
    }
    else
    {
      invalid_data_ = 0;
      //Quaternion message comes in as a Yaw (z) pitch (y) Roll (x) format
      if (params_.tf_ned_to_enu) {
        // If we want the orientation to be based on the reference label on the imu
        tf2::Quaternion tf2_quat(q[0], q[1], q[2], q[3]);
        geometry_msgs::msg::Quaternion quat_msg;

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
        geometry_msgs::msg::Quaternion quat_msg;

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
  RCLCPP_WARN(get_logger(), "IMU invalid data, discarding message");
  return false;
}

bool Vectornav::fill_mag_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::MagneticField & mag_msg, const rclcpp::Time & ros_time)
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
  return true;
}

bool Vectornav::fill_temp_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::Temperature & temp_msg, const rclcpp::Time & ros_time)
{
  temp_msg.header.stamp = ros_time;
  temp_msg.header.frame_id = params_.imu_frame_id;
  if (cd.hasTemperature()) {
    float const temp { cd.temperature() };
    temp_msg.temperature = static_cast<double>(temp);
  }
  return true;
}

bool Vectornav::fill_pres_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::FluidPressure & pres_msg, const rclcpp::Time & ros_time)
{
  pres_msg.header.stamp = ros_time;
  pres_msg.header.frame_id = params_.imu_frame_id;
  if (cd.hasPressure()) {
    float const pres { cd.pressure() };
    pres_msg.fluid_pressure = static_cast<double>(pres);
  }
  return true;
}

bool Vectornav::fill_gps_msg(vn::sensors::CompositeData & cd, sensor_msgs::msg::NavSatFix & gps_msg, const rclcpp::Time & ros_time)
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
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      } else {
        // position not detected
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }

      // add the type of covariance to the gps message
      gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
  }
  return true;
}

bool Vectornav::fill_odom_msg(vn::sensors::CompositeData & cd, nav_msgs::msg::Odometry & odom_msg, const rclcpp::Time & ros_time)
{
  // Check with vectornav different of VN-100
  odom_msg.header.stamp = ros_time;
  odom_msg.child_frame_id = params_.imu_frame_id;
  odom_msg.header.frame_id = params_.map_frame_id;

  if (cd.hasPositionEstimatedEcef()) {
    // add position as earth fixed frame
    vn::math::vec3d pos { cd.positionEstimatedEcef() };

    if (!initial_position_set_) {
      RCLCPP_INFO(get_logger(), "Set initial position to %f %f %f", pos[0], pos[1], pos[2]);
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

bool Vectornav::fill_ins_msg(vn::sensors::CompositeData & cd, vectornav::msg::Ins & ins_msg, const rclcpp::Time & ros_time)
{
  // Check with vectornav different of VN-100
  ins_msg.header.stamp = ros_time;
  ins_msg.header.frame_id = params_.imu_frame_id;

  if (cd.hasInsStatus()) {
    auto const insStatus { cd.insStatus() };
    ins_msg.ins_status = static_cast<uint16_t>(insStatus);
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
    std::memcpy(&ins_msg.utc_time, utcTimeBytes, 8);
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
    ins_msg.ned_vel_x = nedVel[0];
    ins_msg.ned_vel_y = nedVel[1];
    ins_msg.ned_vel_z = nedVel[2];
  }

  if (cd.hasAttitudeUncertainty()) {
    vn::math::vec3f const attUncertainty { cd.attitudeUncertainty() };
    ins_msg.att_uncertainty[0] = attUncertainty[0];
    ins_msg.att_uncertainty[1] = attUncertainty[1];
    ins_msg.att_uncertainty[2] = attUncertainty[2];
  }

  if (cd.hasPositionUncertaintyEstimated()) {
    ins_msg.pos_uncertainty = cd.positionUncertaintyEstimated();
  }

  if (cd.hasVelocityUncertaintyEstimated()) {
    ins_msg.vel_uncertainty = cd.velocityUncertaintyEstimated();
  }
  return true;
}

}  // namespace vectornav
