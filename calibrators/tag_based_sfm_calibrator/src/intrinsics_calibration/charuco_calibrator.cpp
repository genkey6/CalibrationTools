// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tag_based_sfm_calibrator/intrinsics_calibration/charuco_calibrator.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace tag_based_sfm_calibrator
{

CharucoBasedCalibrator::CharucoBasedCalibrator(
  int squares_x, int squares_y, double square_length, double marker_length,
  const std::string & dictionary_name, bool use_tangent_distortion,
  int num_radial_distortion_coeffs, bool debug)
: IntrinsicsCalibrator(use_tangent_distortion, num_radial_distortion_coeffs, debug),
  squares_x_(squares_x),
  squares_y_(squares_y),
  square_length_(square_length),
  marker_length_(marker_length)
{
  // Validate parameters
  if (square_length <= marker_length) {
    throw std::invalid_argument(
      "ChArUco board square_length must be greater than marker_length");
  }

  // Create dictionary
  dictionary_ = createDictionary(dictionary_name);

  // Create ChArUco board (using older API)
  board_ = cv::aruco::CharucoBoard::create(
    squares_x_, squares_y_, static_cast<float>(square_length_),
    static_cast<float>(marker_length_), dictionary_);

  // Create detector parameters
  detector_params_ = cv::aruco::DetectorParameters::create();
}

cv::Ptr<cv::aruco::Dictionary> CharucoBasedCalibrator::createDictionary(
  const std::string & dictionary_name)
{
  // Map string names to OpenCV ArUco dictionary enums
  if (dictionary_name == "DICT_4X4_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  } else if (dictionary_name == "DICT_4X4_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  } else if (dictionary_name == "DICT_4X4_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  } else if (dictionary_name == "DICT_4X4_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
  } else if (dictionary_name == "DICT_5X5_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  } else if (dictionary_name == "DICT_5X5_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
  } else if (dictionary_name == "DICT_5X5_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  } else if (dictionary_name == "DICT_5X5_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
  } else if (dictionary_name == "DICT_6X6_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  } else if (dictionary_name == "DICT_6X6_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
  } else if (dictionary_name == "DICT_6X6_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  } else if (dictionary_name == "DICT_6X6_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  } else if (dictionary_name == "DICT_7X7_50") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
  } else if (dictionary_name == "DICT_7X7_100") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
  } else if (dictionary_name == "DICT_7X7_250") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
  } else if (dictionary_name == "DICT_7X7_1000") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
  } else if (dictionary_name == "DICT_ARUCO_ORIGINAL") {
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  } else {
    // Default to DICT_4X4_50
    RCLCPP_WARN(
      rclcpp::get_logger("charuco_calibrator"),
      "Unknown dictionary name: %s, using DICT_4X4_50 as default", dictionary_name.c_str());
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }
}

void CharucoBasedCalibrator::extractCalibrationPoints()
{
  size_.height = -1;
  size_.width = -1;

  for (std::size_t i = 0; i < calibration_image_file_names_.size(); ++i) {
    cv::Mat grayscale_img = cv::imread(
      calibration_image_file_names_[i], cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION);

    assert(size_.height == -1 || size_.height == grayscale_img.rows);
    assert(size_.width == -1 || size_.width == grayscale_img.cols);
    size_ = grayscale_img.size();

    RCLCPP_INFO(
      rclcpp::get_logger("charuco_calibrator"), "Processing image %lu: %s (size: %dx%d)", i,
      calibration_image_file_names_[i].c_str(), grayscale_img.cols, grayscale_img.rows);

    // First detect ArUco markers
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(grayscale_img, dictionary_, marker_corners, marker_ids, detector_params_);

    // Then interpolate ChArUco corners
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;

    if (marker_ids.size() > 0) {
      cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, grayscale_img, board_,
        charuco_corners, charuco_ids);
    }

    // Need at least 6 points for OpenCV's DLT pose estimation inside calibrateCamera
    constexpr std::size_t min_charuco_corners = 6;

    if (charuco_corners.size() >= min_charuco_corners) {
      RCLCPP_INFO(
        rclcpp::get_logger("charuco_calibrator"),
        "Found %lu ChArUco corners in image %lu", charuco_corners.size(), i);

      // Store the valid detection
      filtered_image_file_names_.push_back(calibration_image_file_names_[i]);
      filtered_charuco_ids_.push_back(charuco_ids);
      filtered_charuco_corners_.push_back(charuco_corners);

      // Convert to 3D object points and 2D image points for calibration
      std::vector<cv::Point3f> obj_points;
      for (size_t j = 0; j < charuco_ids.size(); ++j) {
        cv::Point3f pt = board_->chessboardCorners[charuco_ids[j]];
        obj_points.push_back(pt);
      }

      object_points_.push_back(obj_points);
      image_points_.push_back(charuco_corners);
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("charuco_calibrator"),
        "Insufficient ChArUco corners detected (%lu < %zu) in image %lu, skipping",
        charuco_corners.size(), min_charuco_corners, i);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("charuco_calibrator"),
    "Successfully processed %lu out of %lu images for calibration",
    filtered_image_file_names_.size(), calibration_image_file_names_.size());
}

void CharucoBasedCalibrator::writeDebugImages(const IntrinsicParameters & intrinsics)
{
  for (std::size_t i = 0; i < filtered_image_file_names_.size(); i++) {
    const std::string input_file_name = filtered_image_file_names_[i];
    std::size_t name_start_pos = input_file_name.find_last_of("/\\");
    std::size_t name_end_pos = input_file_name.find_last_of(".");
    std::string raw_name =
      input_file_name.substr(name_start_pos + 1, name_end_pos - name_start_pos - 1);

    std::string distorted_image_name = raw_name + "_charuco_distorted.jpg";
    std::string undistorted_image_name = raw_name + "_charuco_undistorted.jpg";

    cv::Mat distorted_img =
      cv::imread(input_file_name, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
    cv::Mat undistorted_img;

    // Draw detected ChArUco corners on the distorted image
    if (filtered_charuco_corners_[i].size() > 0) {
      cv::aruco::drawDetectedCornersCharuco(
        distorted_img, filtered_charuco_corners_[i], filtered_charuco_ids_[i],
        cv::Scalar(0, 255, 0));
    }

    // Undistort the image
    cv::undistort(
      distorted_img, undistorted_img, intrinsics.camera_matrix, intrinsics.dist_coeffs,
      intrinsics.undistorted_camera_matrix);

    // Project the object points onto the undistorted image
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
      object_points_[i], rvecs_[i], tvecs_[i], intrinsics.undistorted_camera_matrix, cv::Mat(),
      projected_points);

    // Draw projected points on the undistorted image
    for (size_t j = 0; j < projected_points.size(); ++j) {
      cv::circle(undistorted_img, projected_points[j], 5, cv::Scalar(255, 0, 255), -1);
      if (j < filtered_charuco_ids_[i].size()) {
        cv::putText(
          undistorted_img, std::to_string(filtered_charuco_ids_[i][j]),
          projected_points[j] + cv::Point2f(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.4,
          cv::Scalar(255, 255, 0), 1);
      }
    }

    cv::imwrite(distorted_image_name, distorted_img);
    cv::imwrite(undistorted_image_name, undistorted_img);

    RCLCPP_INFO(
      rclcpp::get_logger("charuco_calibrator"),
      "Wrote debug images: %s, %s", distorted_image_name.c_str(),
      undistorted_image_name.c_str());
  }
}

}  // namespace tag_based_sfm_calibrator
