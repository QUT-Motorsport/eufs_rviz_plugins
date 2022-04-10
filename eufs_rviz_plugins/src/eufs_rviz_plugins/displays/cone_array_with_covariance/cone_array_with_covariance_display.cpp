#include <Eigen/Eigen>
#include "eufs_rviz_plugins/displays/cone_array_with_covariance/cone_array_with_covariance_display.hpp"  // NOLINT

namespace eufs_rviz_plugins {
namespace displays {

ConeArrayWithCovarianceDisplay::ConeArrayWithCovarianceDisplay()
    : rviz_common::RosTopicDisplay<eufs_msgs::msg::ConeArrayWithCovariance>()
    , id_(0)
    , marker_common_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this)) { }

void ConeArrayWithCovarianceDisplay::onInitialize() {
  RTDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);

  topic_property_->setValue("/cones");
  topic_property_->setDescription("eufs_msgs::msg::ConeArrayWithCovariance topic to subscribe to.");

  initMarkers();
}

void ConeArrayWithCovarianceDisplay::load(
    const rviz_common::Config &config) {
  Display::load(config);
  marker_common_->load(config);
}

void ConeArrayWithCovarianceDisplay::processMessage(
    eufs_msgs::msg::ConeArrayWithCovariance::ConstSharedPtr msg) {
  delete_all_marker_.header = msg->header;
  delete_all_marker_.id = id_;
  marker_array_.markers.push_back(delete_all_marker_);

  marker_common_->addMessage(std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

  marker_array_.markers.clear();

  setMarkerArray(msg);
  marker_common_->addMessage(std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

  marker_array_.markers.clear();
}

void ConeArrayWithCovarianceDisplay::update(
    float wall_dt,
    float ros_dt) {
  marker_common_->update(wall_dt, ros_dt);
}

void ConeArrayWithCovarianceDisplay::reset() {
  RosTopicDisplay::reset();
  marker_common_->clearMarkers();
}

void ConeArrayWithCovarianceDisplay::initMarkers() {
  delete_all_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

  blue_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  blue_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  blue_cone_marker_.pose.orientation.x = 0.0;
  blue_cone_marker_.pose.orientation.y = 0.0;
  blue_cone_marker_.pose.orientation.z = 0.0;
  blue_cone_marker_.pose.orientation.w = 1.0;
  blue_cone_marker_.scale.x = 1.0;
  blue_cone_marker_.scale.y = 1.0;
  blue_cone_marker_.scale.z = 1.0;
  blue_cone_marker_.mesh_resource = "package://eufs_rviz_plugins/meshes/cone.dae";
  blue_cone_marker_.color.r = 0.0;
  blue_cone_marker_.color.g = 0.0;
  blue_cone_marker_.color.b = 1.0;
  blue_cone_marker_.color.a = 1.0;
  blue_cone_marker_.ns = "cone";

  yellow_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  yellow_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  yellow_cone_marker_.pose.orientation.x = 0.0;
  yellow_cone_marker_.pose.orientation.y = 0.0;
  yellow_cone_marker_.pose.orientation.z = 0.0;
  yellow_cone_marker_.pose.orientation.w = 1.0;
  yellow_cone_marker_.scale.x = 1.0;
  yellow_cone_marker_.scale.y = 1.0;
  yellow_cone_marker_.scale.z = 1.0;
  yellow_cone_marker_.mesh_resource = "package://eufs_rviz_plugins/meshes/cone.dae";
  yellow_cone_marker_.color.r = 1.0;
  yellow_cone_marker_.color.g = 1.0;
  yellow_cone_marker_.color.b = 0.0;
  yellow_cone_marker_.color.a = 1.0;
  yellow_cone_marker_.ns = "cone";

  orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  orange_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  orange_cone_marker_.pose.orientation.x = 0.0;
  orange_cone_marker_.pose.orientation.y = 0.0;
  orange_cone_marker_.pose.orientation.z = 0.0;
  orange_cone_marker_.pose.orientation.w = 1.0;
  orange_cone_marker_.scale.x = 1.0;
  orange_cone_marker_.scale.y = 1.0;
  orange_cone_marker_.scale.z = 1.0;
  orange_cone_marker_.mesh_resource = "package://eufs_rviz_plugins/meshes/cone.dae";
  orange_cone_marker_.color.r = 1.0;
  orange_cone_marker_.color.g = 0.549;
  orange_cone_marker_.color.b = 0.0;
  orange_cone_marker_.color.a = 1.0;
  orange_cone_marker_.ns = "cone";

  big_orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  big_orange_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  big_orange_cone_marker_.pose.orientation.x = 0.0;
  big_orange_cone_marker_.pose.orientation.y = 0.0;
  big_orange_cone_marker_.pose.orientation.z = 0.0;
  big_orange_cone_marker_.pose.orientation.w = 1.0;
  big_orange_cone_marker_.scale.x = 1.0;
  big_orange_cone_marker_.scale.y = 1.0;
  big_orange_cone_marker_.scale.z = 1.0;
  big_orange_cone_marker_.mesh_resource = "package://eufs_rviz_plugins/meshes/big_cone.dae";
  big_orange_cone_marker_.color.r = 1.0;
  big_orange_cone_marker_.color.g = 0.271;
  big_orange_cone_marker_.color.b = 0.0;
  big_orange_cone_marker_.color.a = 1.0;
  big_orange_cone_marker_.ns = "cone";

  unknown_cone_marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  unknown_cone_marker_.pose.orientation.x = 0.0;
  unknown_cone_marker_.pose.orientation.y = 0.0;
  unknown_cone_marker_.pose.orientation.z = 0.0;
  unknown_cone_marker_.pose.orientation.w = 1.0;
  unknown_cone_marker_.scale.x = 1.0;
  unknown_cone_marker_.scale.y = 1.0;
  unknown_cone_marker_.scale.z = 1.0;
  unknown_cone_marker_.mesh_resource = "package://eufs_rviz_plugins/meshes/cone.dae";
  unknown_cone_marker_.color.r = 0.0;
  unknown_cone_marker_.color.g = 1.0;
  unknown_cone_marker_.color.b = 0.0;
  unknown_cone_marker_.color.a = 1.0;
  unknown_cone_marker_.ns = "cone";

  covariance_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  covariance_marker_.pose.orientation.x = 0.0;
  covariance_marker_.pose.orientation.y = 0.0;
  covariance_marker_.pose.orientation.z = 0.0;
  covariance_marker_.pose.orientation.w = 1.0;
  covariance_marker_.scale.x = 1.0;
  covariance_marker_.scale.y = 1.0;
  covariance_marker_.scale.z = 1.0;
  covariance_marker_.color.r = 1.0;
  covariance_marker_.color.g = 0.271;
  covariance_marker_.color.b = 0.271;
  covariance_marker_.color.a = 0.7;
  covariance_marker_.ns = "covariance";
}

void ConeArrayWithCovarianceDisplay::setConeMarker(
    const eufs_msgs::msg::ConeWithCovariance &cone,
    const std_msgs::msg::Header &header,
    const int &id,
    visualization_msgs::msg::Marker *marker) {
  marker->id = id;
  marker->header = header;
  marker->pose.position.x = cone.point.x;
  marker->pose.position.y = cone.point.y;
  marker->pose.position.z = cone.point.z;
}

void ConeArrayWithCovarianceDisplay::setCovarianceMarker(
    const eufs_msgs::msg::ConeWithCovariance &cone,
    const std_msgs::msg::Header &header,
    const int &id) {
  // https://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
  covariance_marker_.id = id;
  covariance_marker_.header = header;
  covariance_marker_.pose.position.x = cone.point.x;
  covariance_marker_.pose.position.y = cone.point.y;
  covariance_marker_.pose.position.z = cone.point.z;

  // Convert the covariance message to a matrix
  Eigen::Matrix2f covariance_matrix;
  covariance_matrix <<
  static_cast<float>(cone.covariance[0]), static_cast<float>(cone.covariance[1]),
  static_cast<float>(cone.covariance[2]), static_cast<float>(cone.covariance[3]);

  // Solve the covariance matrix for eigenvectors and eigenvalues
  // Although a positive semi-definite matrix cannot have complex eigenstuff, we discard any
  // imaginary parts that might have appeared by computational errors
  Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;
  eigensolver.compute(covariance_matrix);
  Eigen::Vector2f eigenvalues = eigensolver.eigenvalues().real();
  Eigen::Matrix2f eigenvectors = eigensolver.eigenvectors().real();

  // Get the rotation matrix from the eigenvectors and place it in 3-D.
  Eigen::Matrix3f rotation_matrix;
  rotation_matrix <<
  eigenvectors(0, 0), eigenvectors(0, 1), 0,
  eigenvectors(1, 0), eigenvectors(1, 1), 0,
  0, 0, 1;

  // Make quaternion from rotation matrix and add it to the marker message
  Eigen::Quaternionf quaternion(rotation_matrix);

  covariance_marker_.pose.orientation.x = quaternion.x();
  covariance_marker_.pose.orientation.y = quaternion.y();
  covariance_marker_.pose.orientation.z = quaternion.z();
  covariance_marker_.pose.orientation.w = quaternion.w();

  // Make the scale from the eigenvalues to be the 95% confidence ellipse
  covariance_marker_.scale.x = 2 * sqrt(5.991 * eigenvalues(0));
  covariance_marker_.scale.y = 2 * sqrt(5.991 * eigenvalues(1));
  covariance_marker_.scale.z = 0.01;
}

void ConeArrayWithCovarianceDisplay::setMarkerArray(
    const eufs_msgs::msg::ConeArrayWithCovariance::ConstSharedPtr &msg) {
  for (const auto &cone : msg->blue_cones) {
    setConeMarker(cone, msg->header, id_, &blue_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    marker_array_.markers.push_back(blue_cone_marker_);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->yellow_cones) {
    setConeMarker(cone, msg->header, id_, &yellow_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    marker_array_.markers.push_back(yellow_cone_marker_);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->orange_cones) {
    setConeMarker(cone, msg->header, id_, &orange_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    marker_array_.markers.push_back(orange_cone_marker_);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->unknown_color_cones) {
    setConeMarker(cone, msg->header, id_, &unknown_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    marker_array_.markers.push_back(unknown_cone_marker_);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->big_orange_cones) {
    setConeMarker(cone, msg->header, id_, &big_orange_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    marker_array_.markers.push_back(big_orange_cone_marker_);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
}

}  // namespace displays
}  // namespace eufs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    eufs_rviz_plugins::displays::ConeArrayWithCovarianceDisplay,
    rviz_common::Display)
