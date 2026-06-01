#include <GeographicLib/UTMUPS.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <string>

namespace
{
bool isValidFix(const sensor_msgs::msg::NavSatFix & msg)
{
  return msg.status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX &&
         std::isfinite(msg.latitude) && std::isfinite(msg.longitude);
}
}  // namespace

class GpsMapProjectorNode : public rclcpp::Node
{
public:
  GpsMapProjectorNode()
  : Node("gps_map_projector_node")
  {
    cartesian_frame_ = this->declare_parameter<std::string>("cartesian_frame", "utm");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_link_frame_ = this->declare_parameter<std::string>("base_link_frame", "base_link");
    gps_fix_topic_ = this->declare_parameter<std::string>("gps_fix_topic", "/gps/fix");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odometry/local");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/odometry/gps_map");
    zero_altitude_ = this->declare_parameter<bool>("zero_altitude", true);
    transform_timeout_sec_ = this->declare_parameter<double>("transform_timeout", 0.5);
    local_odom_timeout_sec_ = this->declare_parameter<double>("local_odom_timeout", 0.5);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    auto gps_sub_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(5));
    auto odom_sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_fix_topic_, gps_sub_qos,
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        this->gpsFixCallback(msg);
      });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_sub_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        this->localOdomCallback(msg);
      });

    gps_map_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));
  }

private:
  void localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    latest_local_odom_ = *msg;
    has_local_odom_ = true;

    if (!msg->child_frame_id.empty()) {
      base_link_frame_ = msg->child_frame_id;
    }
  }

  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr & msg)
  {
    if (!isValidFix(*msg)) {
      return;
    }

    if (!hasLocalOdomFor(*msg)) {
      return;
    }

    int zone = 0;
    bool northp = true;
    double utm_x = 0.0;
    double utm_y = 0.0;
    const double altitude = std::isfinite(msg->altitude) ? msg->altitude : 0.0;

    try {
      GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, utm_x, utm_y);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert GPS fix to UTM: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose_utm;
    pose_utm.header.frame_id = cartesian_frame_;
    pose_utm.header.stamp = msg->header.stamp;
    pose_utm.pose.position.x = utm_x;
    pose_utm.pose.position.y = utm_y;
    pose_utm.pose.position.z = zero_altitude_ ? 0.0 : altitude;
    pose_utm.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped pose_map;
    try {
      if (!tf_buffer_->canTransform(
          map_frame_, cartesian_frame_, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_sec_))) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Transform %s -> %s is not available yet.",
          cartesian_frame_.c_str(), map_frame_.c_str());
        return;
      }

      const auto transform = tf_buffer_->lookupTransform(map_frame_, cartesian_frame_, tf2::TimePointZero);
      tf2::doTransform(pose_utm, pose_map, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to project GPS fix into %s: %s", map_frame_.c_str(), ex.what());
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_link_frame_;
    odom.pose.pose.position = pose_map.pose.position;
    odom.pose.pose.position.z = zero_altitude_ ? 0.0 : odom.pose.pose.position.z;
    odom.pose.pose.orientation = latest_local_odom_.pose.pose.orientation;
    odom.twist = latest_local_odom_.twist;

    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        odom.pose.covariance[row * 6 + col] = msg->position_covariance[row * 3 + col];
      }
    }

    for (size_t row = 3; row < 6; ++row) {
      for (size_t col = 3; col < 6; ++col) {
        odom.pose.covariance[row * 6 + col] = latest_local_odom_.pose.covariance[row * 6 + col];
      }
    }

    odom.pose.covariance[2 * 6 + 2] = zero_altitude_ ? 1e-6 : odom.pose.covariance[2 * 6 + 2];
    odom.twist.covariance = latest_local_odom_.twist.covariance;

    gps_map_pub_->publish(odom);
  }

  bool hasLocalOdomFor(const sensor_msgs::msg::NavSatFix & msg)
  {
    if (!has_local_odom_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for local odometry from %s before publishing %s.",
        odom_topic_.c_str(), output_topic_.c_str());
      return false;
    }

    if (local_odom_timeout_sec_ <= 0.0) {
      return true;
    }

    const auto gps_time = rclcpp::Time(msg.header.stamp);
    const auto odom_time = rclcpp::Time(latest_local_odom_.header.stamp);
    const auto delta = gps_time > odom_time ? (gps_time - odom_time) : (odom_time - gps_time);

    if (delta.seconds() > local_odom_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Local odometry is too old for GPS projection. delta=%.3f sec", delta.seconds());
      return false;
    }

    return true;
  }

  std::string cartesian_frame_;
  std::string map_frame_;
  std::string base_link_frame_;
  std::string gps_fix_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  bool zero_altitude_;
  double transform_timeout_sec_;
  bool has_local_odom_ = false;
  double local_odom_timeout_sec_;
  nav_msgs::msg::Odometry latest_local_odom_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsMapProjectorNode>());
  rclcpp::shutdown();
  return 0;
}