#pragma once

#include <optional>
#include <string>
#include <vector>

#include <opencv2/aruco/charuco.hpp>

namespace board_recognition
{
struct Pose
{
  cv::Vec3d rotation;
  cv::Vec3d translation;
};

struct Detection
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> borders;
  std::optional<Pose> pose;
};

class BoardDetector
{
public:
  explicit BoardDetector(const std::string &config_path);
  Detection detect(const cv::Mat &bgr, const cv::Mat &camera, const cv::Mat &distortion) const;
  void draw_axes(cv::Mat &image, const Pose &pose, const cv::Mat &camera,
    const cv::Mat &distortion) const;

private:
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  cv::Vec3d center_;
  float axis_length_;
};
}  // namespace board_recognition
