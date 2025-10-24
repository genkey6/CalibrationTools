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

#ifndef TAG_BASED_SFM_CALIBRATOR__INTRINSICS_CALIBRATION__CHARUCO_CALIBRATOR_HPP_
#define TAG_BASED_SFM_CALIBRATOR__INTRINSICS_CALIBRATION__CHARUCO_CALIBRATOR_HPP_

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <tag_based_sfm_calibrator/intrinsics_calibration/intrinsics_calibrator.hpp>
#include <tag_based_sfm_calibrator/types.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tag_based_sfm_calibrator
{

class CharucoBasedCalibrator : public IntrinsicsCalibrator
{
public:
  CharucoBasedCalibrator(
    int squares_x, int squares_y, double square_length, double marker_length,
    const std::string & dictionary_name, bool use_tangent_distortion,
    int num_radial_distortion_coeffs, bool debug = false);

protected:
  void extractCalibrationPoints() override;
  void writeDebugImages(const IntrinsicParameters & intrinsics) override;

private:
  cv::Ptr<cv::aruco::Dictionary> createDictionary(const std::string & dictionary_name);

  std::vector<std::string> filtered_image_file_names_;
  std::vector<std::vector<int>> filtered_charuco_ids_;
  std::vector<std::vector<cv::Point2f>> filtered_charuco_corners_;

  int squares_x_;
  int squares_y_;
  double square_length_;
  double marker_length_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

}  // namespace tag_based_sfm_calibrator

#endif  // TAG_BASED_SFM_CALIBRATOR__INTRINSICS_CALIBRATION__CHARUCO_CALIBRATOR_HPP_