#include <GeographicLib/UTMUPS.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

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

class GeoMapAnchorNode : public rclcpp::Node
{
public:
  GeoMapAnchorNode()
  : Node("geo_map_anchor_node")
  {
    cartesian_frame_ = this->declare_parameter<std::string>("cartesian_frame", "utm");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    gps_fix_topic_ = this->declare_parameter<std::string>("gps_fix_topic", "/gps/fix");
    use_manual_datum_ = this->declare_parameter<bool>("use_manual_datum", false);
    datum_latitude_ = this->declare_parameter<double>("datum_latitude", 0.0);
    datum_longitude_ = this->declare_parameter<double>("datum_longitude", 0.0);
    datum_altitude_ = this->declare_parameter<double>("datum_altitude", 0.0);
    datum_yaw_ = this->declare_parameter<double>("datum_yaw", 0.0);
    zero_altitude_ = this->declare_parameter<bool>("zero_altitude", true);

    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    if (use_manual_datum_) {
      publishAnchor(datum_latitude_, datum_longitude_, datum_altitude_, "manual datum");
      return;
    }

    auto gps_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(5));
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_fix_topic_, gps_qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        this->gpsFixCallback(*msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for first valid GPS fix to anchor %s relative to %s.",
      map_frame_.c_str(), cartesian_frame_.c_str());
  }

private:
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix & msg)
  {
    if (published_ || !isValidFix(msg)) {
      return;
    }

    const double altitude = std::isfinite(msg.altitude) ? msg.altitude : 0.0;
    publishAnchor(msg.latitude, msg.longitude, altitude, "first GPS fix");
    gps_sub_.reset();
  }

  void publishAnchor(double latitude, double longitude, double altitude, const std::string & source)
  {
    int zone = 0;
    bool northp = true;
    double utm_x = 0.0;
    double utm_y = 0.0;

    try {
      GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_x, utm_y);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to convert datum to UTM: %s", ex.what());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = cartesian_frame_;
    transform.child_frame_id = map_frame_;
    transform.transform.translation.x = utm_x;
    transform.transform.translation.y = utm_y;
    transform.transform.translation.z = zero_altitude_ ? 0.0 : altitude;

    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, datum_yaw_);
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();

    broadcaster_->sendTransform(transform);
    published_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Published fixed %s -> %s anchor from %s. datum=(%.8f, %.8f), datum_utm=(%.3f, %.3f), applied_translation=(%.3f, %.3f), datum_yaw=%.6f rad, zone=%d%s",
      cartesian_frame_.c_str(), map_frame_.c_str(), source.c_str(),
      latitude, longitude, utm_x, utm_y,
      transform.transform.translation.x, transform.transform.translation.y,
      datum_yaw_, zone, northp ? "N" : "S");
  }

  std::string cartesian_frame_;
  std::string map_frame_;
  std::string gps_fix_topic_;
  bool use_manual_datum_;
  double datum_latitude_;
  double datum_longitude_;
  double datum_altitude_;
  double datum_yaw_;
  bool zero_altitude_;
  bool published_ = false;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoMapAnchorNode>());
  rclcpp::shutdown();
  return 0;
}