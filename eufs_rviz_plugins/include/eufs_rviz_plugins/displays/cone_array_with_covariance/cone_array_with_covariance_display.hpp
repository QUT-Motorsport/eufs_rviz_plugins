#ifndef EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_CONE_ARRAY_WITH_COVARIANCE_CONE_ARRAY_WITH_COVARIANCE_DISPLAY_HPP_  // NOLINT
#define EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_CONE_ARRAY_WITH_COVARIANCE_CONE_ARRAY_WITH_COVARIANCE_DISPLAY_HPP_  // NOLINT

#include <driverless_msgs/msg/cone_detection_stamped.hpp>
#include <driverless_msgs/msg/cone_with_covariance.hpp>
#include <memory>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rviz_default_plugins/visibility_control.hpp"

namespace eufs_rviz_plugins {
namespace displays {

enum ConeColorOption { CONE = 0, FLAT = 1 };

class ConeArrayWithCovarianceDisplay : public rviz_common::RosTopicDisplay<driverless_msgs::msg::ConeDetectionStamped> {
    Q_OBJECT

   public:
    ConeArrayWithCovarianceDisplay();

    void onInitialize() override;

    void load(const rviz_common::Config &config) override;

    void update(float wall_dt, float ros_dt) override;

    void reset() override;

   private Q_SLOTS:
    void updateColorOption();

   private:
    void initMarkers();

    void setConeMarker(const driverless_msgs::msg::Cone &cone, const std_msgs::msg::Header &header,
                       const int &id, visualization_msgs::msg::Marker *marker);

    visualization_msgs::msg::Marker getColoredMarker(visualization_msgs::msg::Marker cone_marker);

    void setCovarianceMarker(const driverless_msgs::msg::ConeWithCovariance &cone, const std_msgs::msg::Header &header,
                             const int &id);

    void setMarkerArray(const driverless_msgs::msg::ConeDetectionStamped::ConstSharedPtr &msg);

    void processMessage(driverless_msgs::msg::ConeDetectionStamped::ConstSharedPtr msg) override;

    int id_;

    ConeColorOption cone_color_option_;

    std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> marker_common_;

    rviz_common::properties::EnumProperty *color_option_property_;
    rviz_common::properties::ColorProperty *color_property_;

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

#endif  // EUFS_RVIZ_PLUGINS_INCLUDE_EUFS_RVIZ_PLUGINS_DISPLAYS_CONE_ARRAY_WITH_COVARIANCE_CONE_ARRAY_WITH_COVARIANCE_DISPLAY_HPP_
        // // NOLINT
