/** VectorNav ROS2 Interface
 *
 * Copyright 2021 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"

using namespace std::chrono_literals;

class VnSensorMsgs: public rclcpp::Node
{
  /// TODO(Dereck): Add _ned topics as an optional feature
  /// TODO(Dereck): Detect when a subscriber is connected but the required data is not provided by
  ///               the sensor.
  /// TODO(Dereck): Improve Multi-message Syncronization

public:
  VnSensorMsgs(const rclcpp::NodeOptions & options) : Node("vn_sensor_msgs", options)
  {
    // Parameters
    declare_parameter < bool > ("dynamic_uncertanity", false);
    declare_parameter < bool > ("gravity_removed_accel", false);

    declare_parameter < std::vector < double >> ("orientation_covariance");
    declare_parameter < std::vector < double >> ("position_covariance");
    declare_parameter < std::vector < double >> ("velocity_covariance");
    declare_parameter < std::vector < double >> ("angular_velocity_covariance");
    declare_parameter < std::vector < double >> ("linear_acceleration_covariance");
    declare_parameter < std::vector < double >> ("magnetic_field_covariance");

    get_parameter("dynamic_uncertanity", dynamic_uncertanity_);
    get_parameter("gravity_removed_accel", gravity_removed_accel_);

    fill_covariance_from_param("orientation_covariance", orientation_covariance_);
    fill_covariance_from_param("position_covariance", position_covariance_);
    fill_covariance_from_param("velocity_covariance", velocity_covariance_);
    fill_covariance_from_param("angular_velocity_covariance", angular_velocity_covariance_);
    fill_covariance_from_param("linear_acceleration_covariance", linear_acceleration_covariance_);
    fill_covariance_from_param("magnetic_field_covariance", magnetic_field_covariance_);
    //
    // Publishers
    //
    // TODO(Dereck): Only publish if data is available from the sensor?
    pub_time_startup_ =
      this->create_publisher < sensor_msgs::msg::TimeReference > ("vectornav/time_startup", 10);
    pub_time_gps_ =
      this->create_publisher < sensor_msgs::msg::TimeReference > ("vectornav/time_gps", 10);
    pub_time_syncin_ =
      this->create_publisher < sensor_msgs::msg::TimeReference > ("vectornav/time_syncin", 10);
    pub_time_pps_ =
      this->create_publisher < sensor_msgs::msg::TimeReference > ("vectornav/time_pps", 10);
    pub_imu_ = this->create_publisher < sensor_msgs::msg::Imu > ("vectornav/imu", 10);
    pub_gnss_ = this->create_publisher < sensor_msgs::msg::NavSatFix > ("vectornav/gnss/ins", 10);
    pub_gnss_raw_ = this->create_publisher < sensor_msgs::msg::NavSatFix >
      ("vectornav/gnss/raw", 10);
    pub_imu_uncompensated_ =
      this->create_publisher < sensor_msgs::msg::Imu > ("vectornav/imu_uncompensated", 10);
    pub_magnetic_ =
      this->create_publisher < sensor_msgs::msg::MagneticField > ("vectornav/magnetic", 10);
    pub_temperature_ =
      this->create_publisher < sensor_msgs::msg::Temperature > ("vectornav/temperature", 10);
    pub_pressure_ =
      this->create_publisher < sensor_msgs::msg::FluidPressure > ("vectornav/pressure", 10);
    pub_velocity_ = this->create_publisher < geometry_msgs::msg::TwistWithCovarianceStamped > (
      "vectornav/velocity_body", 10);
    pub_pose_ =
      this->create_publisher < geometry_msgs::msg::PoseWithCovarianceStamped >
      ("vectornav/pose", 10);

    //
    // Subscribers
    //
    auto sub_vn_common_cb = std::bind(&VnSensorMsgs::sub_vn_common, this, std::placeholders::_1);
    sub_vn_common_ = this->create_subscription < vectornav_msgs::msg::CommonGroup > (
      "vectornav/raw/common", 10, sub_vn_common_cb);

    auto sub_vn_time_cb = std::bind(&VnSensorMsgs::sub_vn_time, this, std::placeholders::_1);
    sub_vn_time_ = this->create_subscription < vectornav_msgs::msg::TimeGroup > (
      "vectornav/raw/time", 10, sub_vn_time_cb);

    auto sub_vn_imu_cb = std::bind(&VnSensorMsgs::sub_vn_imu, this, std::placeholders::_1);
    sub_vn_imu_ = this->create_subscription < vectornav_msgs::msg::ImuGroup > (
      "vectornav/raw/imu", 10, sub_vn_imu_cb);

    auto sub_vn_gps_cb = std::bind(&VnSensorMsgs::sub_vn_gps, this, std::placeholders::_1);
    sub_vn_gps_ = this->create_subscription < vectornav_msgs::msg::GpsGroup > (
      "vectornav/raw/gps", 10, sub_vn_gps_cb);

    auto sub_vn_attitude_cb =
      std::bind(&VnSensorMsgs::sub_vn_attitude, this, std::placeholders::_1);
    sub_vn_attitude_ = this->create_subscription < vectornav_msgs::msg::AttitudeGroup > (
      "vectornav/raw/attitude", 10, sub_vn_attitude_cb);

    auto sub_vn_ins_cb = std::bind(&VnSensorMsgs::sub_vn_ins, this, std::placeholders::_1);
    sub_vn_ins_ = this->create_subscription < vectornav_msgs::msg::InsGroup > (
      "vectornav/raw/ins", 10, sub_vn_ins_cb);

    auto sub_vn_gps2_cb = std::bind(&VnSensorMsgs::sub_vn_gps2, this, std::placeholders::_1);
    sub_vn_gps2_ = this->create_subscription < vectornav_msgs::msg::GpsGroup > (
      "vectornav/raw/gps2", 10, sub_vn_gps2_cb);
  }

private:
  /** Convert VN common group data to ROS2 standard message types
   *
   */
  void sub_vn_common(const vectornav_msgs::msg::CommonGroup::SharedPtr msg_in) {}

  /** Convert VN time group data to ROS2 standard message types
   *
   */
  void sub_vn_time(const vectornav_msgs::msg::TimeGroup::SharedPtr msg_in) const {
    // Time Reference (Startup)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_in->header;

      msg.time_ref.sec = std::floor(msg_in->timestartup / 1e9);
      msg.time_ref.nanosec = msg_in->timestartup - (msg.time_ref.sec * 1e9);

      msg.source = "startup";

      pub_time_startup_->publish(msg);
    }

    // Time Reference (GPS)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_in->header;

      msg.time_ref.sec = std::floor(msg_in->timegps / 1e9);
      msg.time_ref.nanosec = msg_in->timegps - (msg.time_ref.sec * 1e9);

      msg.source = "gps";

      pub_time_gps_->publish(msg);
    }

    // Time Reference (SyncIn)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_in->header;

      msg.time_ref.sec = std::floor(msg_in->timesyncin / 1e9);
      msg.time_ref.nanosec = msg_in->timesyncin - (msg.time_ref.sec * 1e9);

      msg.source = "syncin";

      pub_time_syncin_->publish(msg);
    }

    // Time Reference (PPS)
    {
      sensor_msgs::msg::TimeReference msg;
      msg.header = msg_in->header;

      msg.time_ref.sec = std::floor(msg_in->timegpspps / 1e9);
      msg.time_ref.nanosec = msg_in->timegpspps - (msg.time_ref.sec * 1e9);

      msg.source = "pps";

      pub_time_pps_->publish(msg);
    }
  }

  /** Convert VN imu group data to ROS2 standard message types
   *
   */
  void sub_vn_imu(const vectornav_msgs::msg::ImuGroup::SharedPtr msg_in) {
    // IMU
    {
      sensor_msgs::msg::Imu msg;
      msg.header = msg_in->header;

      msg.orientation = quaternion_;

      angular_rate_ = msg_in->angularrate;
      msg.angular_velocity = angular_rate_;

      if (!gravity_removed_accel_) {
        linear_accel_  = msg_in->accel;
      }
      msg.linear_acceleration = linear_accel_;

      msg.orientation_covariance = orientation_covariance_;
      msg.angular_velocity_covariance = angular_velocity_covariance_;
      msg.linear_acceleration_covariance = linear_acceleration_covariance_;

      pub_imu_->publish(msg);
    }

    // IMU (Uncompensated)
    {
      sensor_msgs::msg::Imu msg;
      msg.header = msg_in->header;
      msg.angular_velocity = msg_in->uncompgyro;
      msg.linear_acceleration = msg_in->uncompaccel;

      fill_covariance_from_param("angular_velocity_covariance", msg.angular_velocity_covariance);
      fill_covariance_from_param(
        "linear_acceleration_covariance", msg.linear_acceleration_covariance);

      pub_imu_uncompensated_->publish(msg);
    }

    // Magnetic Field
    {
      sensor_msgs::msg::MagneticField msg;
      msg.header = msg_in->header;
      msg.magnetic_field = msg_in->mag;

      msg.magnetic_field_covariance = magnetic_field_covariance_;

      pub_magnetic_->publish(msg);
    }

    // Temperature
    {
      sensor_msgs::msg::Temperature msg;
      msg.header = msg_in->header;
      msg.temperature = msg_in->temp;

      pub_temperature_->publish(msg);
    }

    // Pressure
    {
      sensor_msgs::msg::FluidPressure msg;
      msg.header = msg_in->header;

      // Convert kPa to Pa
      msg.fluid_pressure = msg_in->pres * 1e3;

      pub_pressure_->publish(msg);
    }

  }

  /** Convert VN gps group data to ROS2 standard message types
   *
   * TODO(Dereck): Consider alternate sync methods
   */
  void sub_vn_gps(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in)
  {
    gps_fix_ = msg_in->fix;
    gps_posu_ = msg_in->posu;

    sensor_msgs::msg::NavSatFix msg;
    msg.header = msg_in->header;

    // Status
    switch (gps_fix_) {
      case vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX:
      case vectornav_msgs::msg::GpsGroup::GPSFIX_TIMEONLY:
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case vectornav_msgs::msg::GpsGroup::GPSFIX_SBAS:
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        break;
      default:
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
    }

    // Position
    msg.latitude = msg_in->poslla.x;
    msg.longitude = msg_in->poslla.y;
    msg.altitude = msg_in->poslla.z;

    // Covariance (Convert NED to ENU)
    gps_covariance_[0] = msg_in->posu.y * msg_in->posu.y;
    gps_covariance_[4] = msg_in->posu.x * msg_in->posu.x;
    gps_covariance_[8] = msg_in->posu.z * msg_in->posu.z;

    msg.position_covariance = gps_covariance_;
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    pub_gnss_raw_->publish(msg);
  }

  /** Convert VN attitude group data to ROS2 standard message types
   *
   */
  void sub_vn_attitude(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg_in)
  {
    if (dynamic_uncertanity_) {
      double r_u = deg2rad(msg_in->ypru.z);
      double p_u = deg2rad(msg_in->ypru.y);
      double y_u = deg2rad(msg_in->ypru.x);
      orientation_covariance_[0] = r_u * r_u;
      orientation_covariance_[4] = p_u * p_u;
      orientation_covariance_[8] = y_u * y_u;
    }
    if (gravity_removed_accel_) {
      linear_accel_ = msg_in->linearaccelbody;
    }

    // Quaternion NED -> ENU
    tf2::Quaternion q, q_ned2enu;
      fromMsg(msg_in->quaternion, q);
      q_ned2enu.setRPY(M_PI, 0.0, M_PI / 2);
    quaternion_ = toMsg(q_ned2enu * q);
  }

  /** Convert VN ins group data to ROS2 standard message types
   *
   */
  void sub_vn_ins(const vectornav_msgs::msg::InsGroup::SharedPtr msg_in)
  {
    ins_velbody_ = msg_in->velbody;
    ins_posecef_ = msg_in->posecef;

    if (dynamic_uncertanity_) {
      double posu_sq = msg_in->posu * msg_in->posu;
      position_covariance_[0] = posu_sq;
      position_covariance_[4] = posu_sq;
      position_covariance_[8] = posu_sq;

      double velu_sq = msg_in->velu * msg_in->velu;
      velocity_covariance_[0] = velu_sq;
      velocity_covariance_[4] = velu_sq;
      velocity_covariance_[8] = velu_sq;
    }

    // GNSS
    {
      sensor_msgs::msg::NavSatFix msg;
      msg.header = msg_in->header;

      // Status
      switch (gps_fix_) {
        case vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX:
        case vectornav_msgs::msg::GpsGroup::GPSFIX_TIMEONLY:
          msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
          break;
        case vectornav_msgs::msg::GpsGroup::GPSFIX_SBAS:
          msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
          break;
        default:
          msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
          break;
      }

      // Position
      msg.latitude = msg_in->poslla.x;
      msg.longitude = msg_in->poslla.y;
      msg.altitude = msg_in->poslla.z;

      // Covariance (Convert NED to ENU)
      // Conversion currently not necessary since all values are the same
      /// TODO(Dereck): Use DOP for better estimate?
      msg.position_covariance = position_covariance_;

      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      pub_gnss_->publish(msg);
    }

    // Pose (ECEF)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header = msg_in->header;
      msg.header.frame_id = "earth";
      msg.pose.pose.position = ins_posecef_;

      // Converts Quaternion in NED to ECEF
      tf2::Quaternion q, q_enu2ecef, q_ned2enu;
      q_ned2enu.setRPY(M_PI, 0.0, M_PI / 2);

      auto latitude = deg2rad(msg_in->poslla.x);
      auto longitude = deg2rad(msg_in->poslla.y);
      q_enu2ecef.setRPY(0.0, latitude, longitude);

      fromMsg(quaternion_, q);

      msg.pose.pose.orientation = toMsg(q_ned2enu * q_enu2ecef * q);

      /// TODO(Dereck): Pose Covariance

      pub_pose_->publish(msg);
    }

    // Velocity
    {
      geometry_msgs::msg::TwistWithCovarianceStamped msg;
      msg.header = msg_in->header;
      msg.twist.twist.linear = ins_velbody_;
      msg.twist.twist.angular = angular_rate_;

      msg.twist.covariance[0] = velocity_covariance_[0];
      msg.twist.covariance[7] = velocity_covariance_[4];
      msg.twist.covariance[14] = velocity_covariance_[8];
      msg.twist.covariance[21] = angular_velocity_covariance_[0];
      msg.twist.covariance[28] = angular_velocity_covariance_[4];
      msg.twist.covariance[35] = angular_velocity_covariance_[8];
      pub_velocity_->publish(msg);
    }
  }

  /** Convert VN gps2 group data to ROS2 standard message types
   *
   */
  void sub_vn_gps2(const vectornav_msgs::msg::GpsGroup::SharedPtr msg_in) const {}

  /** Copy a covariance matrix array from a parameter into a msg array
   *
   * If a single value is provided, this will set the diagonal values
   * If three values are provided, this will set the diagonal values
   * If nine values are provided, this will fill the matrix
   *
   * \param param_name Name of the parameter to read
   * \param array Array to fill
   */
  void fill_covariance_from_param(std::string param_name, std::array < double, 9 > & array) const
  {
    auto covariance = get_parameter(param_name).as_double_array();

    auto length = covariance.size();
    switch (length) {
      case 1:
        array[0] = covariance[0];
        array[4] = covariance[0];
        array[8] = covariance[0];
        break;

      case 3:
        array[0] = covariance[0];
        array[4] = covariance[1];
        array[8] = covariance[2];
        break;

      case 9:
        std::copy(covariance.begin(), covariance.end(), array.begin());
        break;

      default:
        RCLCPP_ERROR(
          get_logger(), "Parameter '%s' length is %zu; expected 1, 3, or 9", param_name.c_str(),
          length);
    }
  }

  /// Convert from DEG to RAD
  inline static double deg2rad(double in) {return in * M_PI / 180.0;}

  //
  // Member Variables
  //

  /// Publishers
  rclcpp::Publisher < sensor_msgs::msg::TimeReference > ::SharedPtr pub_time_startup_;
  rclcpp::Publisher < sensor_msgs::msg::TimeReference > ::SharedPtr pub_time_gps_;
  rclcpp::Publisher < sensor_msgs::msg::TimeReference > ::SharedPtr pub_time_syncin_;
  rclcpp::Publisher < sensor_msgs::msg::TimeReference > ::SharedPtr pub_time_pps_;
  rclcpp::Publisher < sensor_msgs::msg::Imu > ::SharedPtr pub_imu_;
  rclcpp::Publisher < sensor_msgs::msg::NavSatFix > ::SharedPtr pub_gnss_, pub_gnss_raw_;
  rclcpp::Publisher < sensor_msgs::msg::Imu > ::SharedPtr pub_imu_uncompensated_;
  rclcpp::Publisher < sensor_msgs::msg::MagneticField > ::SharedPtr pub_magnetic_;
  rclcpp::Publisher < sensor_msgs::msg::Temperature > ::SharedPtr pub_temperature_;
  rclcpp::Publisher < sensor_msgs::msg::FluidPressure > ::SharedPtr pub_pressure_;
  rclcpp::Publisher < geometry_msgs::msg::TwistWithCovarianceStamped > ::SharedPtr pub_velocity_;
  rclcpp::Publisher < geometry_msgs::msg::PoseWithCovarianceStamped > ::SharedPtr pub_pose_;

  /// Subscribers
  rclcpp::Subscription < vectornav_msgs::msg::CommonGroup > ::SharedPtr sub_vn_common_;
  rclcpp::Subscription < vectornav_msgs::msg::TimeGroup > ::SharedPtr sub_vn_time_;
  rclcpp::Subscription < vectornav_msgs::msg::ImuGroup > ::SharedPtr sub_vn_imu_;
  rclcpp::Subscription < vectornav_msgs::msg::GpsGroup > ::SharedPtr sub_vn_gps_;
  rclcpp::Subscription < vectornav_msgs::msg::AttitudeGroup > ::SharedPtr sub_vn_attitude_;
  rclcpp::Subscription < vectornav_msgs::msg::InsGroup > ::SharedPtr sub_vn_ins_;
  rclcpp::Subscription < vectornav_msgs::msg::GpsGroup > ::SharedPtr sub_vn_gps2_;

  // Parameters
  bool dynamic_uncertanity_;
  bool gravity_removed_accel_;

  std::array < double, 9 > orientation_covariance_ {};
  std::array < double, 9 > position_covariance_ {};
  std::array < double, 9 > velocity_covariance_ {};
  std::array < double, 9 > angular_velocity_covariance_ {};
  std::array < double, 9 > linear_acceleration_covariance_ {};
  std::array < double, 9 > magnetic_field_covariance_ {};
  std::array < double, 9 > gps_covariance_ {};

  /// TODO(Dereck): Find default covariance values

  // State Vars
  uint8_t gps_fix_ = vectornav_msgs::msg::GpsGroup::GPSFIX_NOFIX;
  geometry_msgs::msg::Vector3 gps_posu_;
  geometry_msgs::msg::Vector3 linear_accel_;
  geometry_msgs::msg::Vector3 angular_rate_;
  geometry_msgs::msg::Quaternion quaternion_;
  geometry_msgs::msg::Vector3 ins_velbody_;
  geometry_msgs::msg::Point ins_posecef_;
};

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(VnSensorMsgs)