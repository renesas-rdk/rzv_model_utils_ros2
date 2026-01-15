// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************
#include "rzv_model_utils_ros2/model_utils.hpp"

#include <algorithm>

namespace rzv_model
{

void UtilsROS::encode_bounding_box_to_poses(
  geometry_msgs::msg::PoseArray & pose_array, const cv::Rect & bbox, const std::string & class_name,
  const int class_id, const float confidence)
{
  float x1 = static_cast<float>(bbox.x);
  float y1 = static_cast<float>(bbox.y);
  float x2 = static_cast<float>(bbox.x + bbox.width);
  float y2 = static_cast<float>(bbox.y + bbox.height);
  float z1 = 0.0f;  // Front plane
  float z2 = 0.0f;  // Back plane (same as front for 2D bbox)

  // Bottom face (z=0)
  std::vector<std::array<float, 3>> corners = {
    {x1, y1, z1},  // 0: bottom-left-front
    {x2, y1, z1},  // 1: bottom-right-front
    {x2, y2, z1},  // 2: bottom-right-back
    {x1, y2, z1},  // 3: bottom-left-back

    // Top face (z=0 for 2D bbox, would be z2 for 3D)
    {x1, y1, z2},  // 4: top-left-front
    {x2, y1, z2},  // 5: top-right-front
    {x2, y2, z2},  // 6: top-right-back
    {x1, y2, z2}   // 7: top-left-back
  };

  // Add each corner as a pose
  for (size_t i = 0; i < corners.size(); i++) {
    const auto & corner = corners[i];
    geometry_msgs::msg::Pose pose;
    pose.position.x = corner[0];
    pose.position.y = corner[1];
    pose.position.z = corner[2];

    // Store metadata in pose orientation components
    // Default quaternion values
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    // Store different parts of metadata in different points' orientations
    switch (i) {
      case 0:  // First point: class_id and confidence
        pose.orientation.x = static_cast<double>(class_id);
        pose.orientation.y = confidence;
        break;

      case 1:  // Second point: class_name characters 0-3
        if (class_name.length() > 0) {
          pose.orientation.x = class_name.length() >= 1 ? static_cast<double>(class_name[0]) : 0.0;
          pose.orientation.y = class_name.length() >= 2 ? static_cast<double>(class_name[1]) : 0.0;
          pose.orientation.z = class_name.length() >= 3 ? static_cast<double>(class_name[2]) : 0.0;
        }
        break;

      case 2:  // Third point: class_name characters 4-7
        if (class_name.length() > 3) {
          pose.orientation.w = class_name.length() >= 4 ? static_cast<double>(class_name[3]) : 1.0;
          pose.orientation.x = class_name.length() >= 5 ? static_cast<double>(class_name[4]) : 0.0;
          pose.orientation.y = class_name.length() >= 6 ? static_cast<double>(class_name[5]) : 0.0;
          pose.orientation.z = class_name.length() >= 7 ? static_cast<double>(class_name[6]) : 0.0;
        }
        break;

      case 3:  // Fourth point: class_name characters 8-11
        if (class_name.length() > 7) {
          pose.orientation.w = class_name.length() >= 8 ? static_cast<double>(class_name[7]) : 1.0;
          pose.orientation.x = class_name.length() >= 9 ? static_cast<double>(class_name[8]) : 0.0;
          pose.orientation.y = class_name.length() >= 10 ? static_cast<double>(class_name[9]) : 0.0;
          pose.orientation.z =
            class_name.length() >= 11 ? static_cast<double>(class_name[10]) : 0.0;
        }
        break;

      case 4:  // Fifth point: class_name characters 12-15
        if (class_name.length() > 11) {
          pose.orientation.w =
            class_name.length() >= 12 ? static_cast<double>(class_name[11]) : 1.0;
          pose.orientation.x =
            class_name.length() >= 13 ? static_cast<double>(class_name[12]) : 0.0;
          pose.orientation.y =
            class_name.length() >= 14 ? static_cast<double>(class_name[13]) : 0.0;
          pose.orientation.z =
            class_name.length() >= 15 ? static_cast<double>(class_name[14]) : 0.0;
        }
        break;

      default:
        // Use default identity quaternion for remaining points
        break;
    }

    pose_array.poses.push_back(pose);
  }
}

void UtilsROS::encode_oriented_bounding_box_to_poses(
  geometry_msgs::msg::PoseArray & pose_array, const cv::RotatedRect & obbox,
  const std::string & class_name, const int class_id, const float confidence)
{
  // Get the 4 corners of the rotated rectangle
  cv::Point2f corners[4];
  obbox.points(corners);

  float z_bottom = 0.0f;
  float z_top = 0.0f;

  std::vector<std::array<float, 3>> corner_points;

  // Add bottom and top face corners
  for (int i = 0; i < 4; i++) {
    corner_points.push_back({corners[i].x, corners[i].y, z_bottom});
  }
  for (int i = 0; i < 4; i++) {
    corner_points.push_back({corners[i].x, corners[i].y, z_top});
  }

  // Add each corner as a pose
  for (size_t i = 0; i < corner_points.size(); i++) {
    const auto & c = corner_points[i];
    geometry_msgs::msg::Pose pose;
    pose.position.x = c[0];
    pose.position.y = c[1];
    pose.position.z = c[2];

    // Store metadata in pose orientation components
    // Default quaternion values
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    // Store different parts of metadata in different points' orientations
    switch (i) {
      case 0:  // First point: class_id and confidence
        pose.orientation.x = class_id;
        pose.orientation.y = confidence;
        break;
      case 1:  // Second point: class_name characters 0-3
        if (!class_name.empty()) {
          pose.orientation.x = class_name.length() >= 1 ? class_name[0] : 0.0;
          pose.orientation.y = class_name.length() >= 2 ? class_name[1] : 0.0;
          pose.orientation.z = class_name.length() >= 3 ? class_name[2] : 0.0;
        }
        break;
      case 2:  // Third point: class_name characters 4-7
        if (class_name.length() > 3) {
          pose.orientation.w = class_name.length() >= 4 ? class_name[3] : 1.0;
          pose.orientation.x = class_name.length() >= 5 ? class_name[4] : 0.0;
          pose.orientation.y = class_name.length() >= 6 ? class_name[5] : 0.0;
          pose.orientation.z = class_name.length() >= 7 ? class_name[6] : 0.0;
        }
        break;
      case 3:  // Fourth point: class_name characters 8-11
        if (class_name.length() > 7) {
          pose.orientation.w = class_name.length() >= 8 ? class_name[7] : 1.0;
          pose.orientation.x = class_name.length() >= 9 ? class_name[8] : 0.0;
          pose.orientation.y = class_name.length() >= 10 ? class_name[9] : 0.0;
          pose.orientation.z = class_name.length() >= 11 ? class_name[10] : 0.0;
        }
        break;
      case 4:  // Fifth point: class_name characters 12-15
        if (class_name.length() > 11) {
          pose.orientation.w = class_name.length() >= 12 ? class_name[11] : 1.0;
          pose.orientation.x = class_name.length() >= 13 ? class_name[12] : 0.0;
          pose.orientation.y = class_name.length() >= 14 ? class_name[13] : 0.0;
          pose.orientation.z = class_name.length() >= 15 ? class_name[14] : 0.0;
        }
        break;
    }

    pose_array.poses.push_back(pose);
  }
}

std::unique_ptr<diagnostic_msgs::msg::DiagnosticStatus> UtilsROS::encode_inference_timing_diagnostic(
  const std::string & message, const float pre_time, const float infer_time, const float post_time)
{
  auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticStatus>();

  msg->name = message;
  msg->level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  msg->message = "Inference timing information";

  msg->values.resize(3);

  msg->values[0].key = "Preprocess Time (ms)";
  msg->values[0].value = std::to_string(pre_time);

  msg->values[1].key = "Inference Time (ms)";
  msg->values[1].value = std::to_string(infer_time);

  msg->values[2].key = "Postprocess Time (ms)";
  msg->values[2].value = std::to_string(post_time);

  return msg;
}

ModelConfig UtilsROS::load_model_config(
  const std::string & package_name, const std::string & model_name)
{
  ModelConfig cfg;

  try {
    const std::string share = ament_index_cpp::get_package_share_directory(package_name);
    const std::string yaml_path = share + "/config/models/models_config.yaml";

    YAML::Node m = YAML::LoadFile(yaml_path)["models"][model_name];

    cfg.model_path = share + "/config/" + m["path"].as<std::string>();

    if (m["names"]) {
      for (const auto & item : m["names"]) {
        cfg.class_names.push_back(item.second.as<std::string>());
      }
    }

  } catch (const std::exception & e) {
    std::cerr << "[Utils] Error loading model '" << model_name << "': " << e.what() << "\n";
  }

  return cfg;
}

ModelConfig UtilsROS::load_model_info(
  const std::string & package_name, const std::string & model_type,
  const std::string & path_override, const std::vector<std::string> & class_names_override)
{
  auto cfg = rzv_model::UtilsROS::load_model_config(package_name, model_type);
  ModelConfig info;
  info.model_path = !path_override.empty() ? path_override : cfg.model_path;
  info.class_names = !class_names_override.empty() ? class_names_override : cfg.class_names;
  return info;
}

}  // namespace rzv_model
