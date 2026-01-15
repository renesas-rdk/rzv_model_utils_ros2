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
#pragma once
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rzv_model
{

// Structure to hold model configuration details
struct ModelConfig
{
  std::string model_path = "";
  std::vector<std::string> class_names = {};
};

class UtilsROS
{
public:
  // Encode bounding box with metadata into a PoseArray for visualization
  static void encode_bounding_box_to_poses(
    geometry_msgs::msg::PoseArray & pose_array, const cv::Rect & bbox,
    const std::string & class_name = "", const int class_id = 0, const float confidence = 0.0f);

  // Encode bounding box with angle and metadata into a PoseArray for visualization
  static void encode_oriented_bounding_box_to_poses(
    geometry_msgs::msg::PoseArray & pose_array, const cv::RotatedRect & obbox,
    const std::string & class_name = "", const int class_id = 0, const float confidence = 0.0f);

  // Encode inference timing information into a DiagnosticStatus message
  static std::unique_ptr<diagnostic_msgs::msg::DiagnosticStatus> encode_inference_timing_diagnostic(
    const std::string & message = "", const float pre_time = 0.0f, const float infer_time = 0.0f,
    const float post_time = 0.0f);

  // Reads the model configuration from the package's YAML file.
  // Loads model settings such as path, class names, and metadata.
  // The caller may override the model path or class names using the optional parameters.
  static ModelConfig load_model_config(
    const std::string & package_name, const std::string & model_name);

  static ModelConfig load_model_info(
    const std::string & package_name, const std::string & model_type,
    const std::string & path_override = "",
    const std::vector<std::string> & class_names_override = {});
};

}  // namespace rzv_model
