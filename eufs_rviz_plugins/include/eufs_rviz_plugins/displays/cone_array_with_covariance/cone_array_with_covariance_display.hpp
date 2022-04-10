#ifndef EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_CONE_ARRAY_WITH_COVARIANCE_CONE_ARRAY_WITH_COVARIANCE_DISPLAY_HPP_  // NOLINT
#define EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_CONE_ARRAY_WITH_COVARIANCE_CONE_ARRAY_WITH_COVARIANCE_DISPLAY_HPP_  // NOLINT

#include <memory>
#include <vector>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"

namespace eufs_rviz_plugins {
namespace displays {

class ConeArrayWithCovarianceDisplay
    : public rviz_common::RosTopicDisplay<eufs_msgs::msg::ConeArrayWithCovariance> {
 public:
  ConeArrayWithCovarianceDisplay();

  void onInitialize() override;

  void load(
      const rviz_common::Config & config) override;

  void update(float wall_dt, float ros_dt) override;

  void reset() override;

 private:
  void initMarkers();

  void setConeMarker(
      const eufs_msgs::msg::ConeWithCovariance &cone,
      const std_msgs::msg::Header &header,
      const int &id,
      visualization_msgs::msg::Marker *marker);

  void setCovarianceMarker(
      const eufs_msgs::msg::ConeWithCovariance &cone,
      const std_msgs::msg::Header &header,
      const int &id);

  void setMarkerArray(
      const eufs_msgs::msg::ConeArrayWithCovariance::ConstSharedPtr &msg);

  void processMessage(
      eufs_msgs::msg::ConeArrayWithCovariance::ConstSharedPtr msg) override;

  int id_;

  std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> marker_common_;

  visualization_msgs::msg::Marker blue_cone_marker_;

  visualization_msgs::msg::Marker yellow_cone_marker_;

  visualization_msgs::msg::Marker orange_cone_marker_;

  visualization_msgs::msg::Marker big_orange_cone_marker_;

  visualization_msgs::msg::Marker unknown_cone_marker_;

  visualization_msgs::msg::Marker covariance_marker_;

  visualization_msgs::msg::Marker delete_all_marker_;

  visualization_msgs::msg::MarkerArray marker_array_;
};

}  // namespace displays
}  // namespace eufs_rviz_plugins

#endif  // EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_CONE_ARRAY_WITH_COVARIANCE_CONE_ARRAY_WITH_COVARIANCE_DISPLAY_HPP_  // NOLINT
