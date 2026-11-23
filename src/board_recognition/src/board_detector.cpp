#include "board_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace board_recognition
{
BoardDetector::BoardDetector(const std::string &config_path)
{
  cv::FileStorage config(config_path, cv::FileStorage::READ);
  if (!config.isOpened()) {
    throw std::runtime_error("Cannot open board configuration: " + config_path);
  }

  std::string dictionary;
  int width = 0;
  int height = 0;
  float square = 0.0F;
  float marker = 0.0F;
  config["dictionary"] >> dictionary;
  config["squares_x"] >> width;
  config["squares_y"] >> height;
  config["square_length_m"] >> square;
  config["marker_length_m"] >> marker;
  if (dictionary != "DICT_4X4_50" || width < 3 || height < 3 ||
    static_cast<long long>(width) * height / 2 > 50 ||
    !std::isfinite(square) || !std::isfinite(marker) || marker <= 0 || square <= marker)
  {
    throw std::runtime_error("Invalid DICT_4X4_50 board geometry in " + config_path);
  }

  board_ = cv::aruco::CharucoBoard::create(width, height, square, marker,
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
  std::vector<int> ids;
  config["marker_ids"] >> ids;
  if (!ids.empty()) {
    const std::unordered_set<int> unique(ids.begin(), ids.end());
    if (ids.size() != board_->ids.size() || unique.size() != ids.size() ||
      *std::min_element(ids.begin(), ids.end()) < 0 || *std::max_element(ids.begin(), ids.end()) >= 50)
    {
      throw std::runtime_error("marker_ids must match the board marker count and use unique IDs 0–49");
    }
    board_->ids = ids;
  }

  // OpenCV 4.5.4 uses the board's bottom-left corner as its origin.
  center_ = {width * square / 2.0, height * square / 2.0, 0.0};
  axis_length_ = square;
  parameters_ = cv::aruco::DetectorParameters::create();
}

Detection BoardDetector::detect(const cv::Mat &bgr, const cv::Mat &camera,
  const cv::Mat &distortion) const
{
  Detection result;
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::aruco::detectMarkers(gray, board_->dictionary, result.borders, result.ids, parameters_);
  if (result.ids.empty() || camera.empty()) {
    return result;
  }

  std::vector<std::vector<cv::Point2f>> board_borders;
  std::vector<int> board_ids;
  for (size_t i = 0; i < result.ids.size(); ++i) {
    if (std::find(board_->ids.begin(), board_->ids.end(), result.ids[i]) != board_->ids.end()) {
      board_borders.push_back(result.borders[i]);
      board_ids.push_back(result.ids[i]);
    }
  }
  if (board_ids.empty()) {
    return result;
  }

  std::vector<cv::Point2f> pixels;
  std::vector<int> corner_ids;
  cv::aruco::interpolateCornersCharuco(board_borders, board_ids, gray, board_, pixels,
    corner_ids, camera, distortion);
  if (corner_ids.size() < 4) {
    return result;
  }

  std::vector<cv::Point3f> points;
  std::vector<cv::Point2f> plane;
  for (const int id : corner_ids) {
    const cv::Point3f corner = board_->chessboardCorners.at(id);
    const float x = static_cast<float>(corner.x - center_[0]);
    const float y = static_cast<float>(corner.y - center_[1]);
    points.emplace_back(x, y, 0.0F);
    plane.emplace_back(x, y);
  }
  std::vector<cv::Point2f> hull;
  cv::convexHull(plane, hull);
  if (hull.size() < 3 || cv::contourArea(hull) <= axis_length_ * axis_length_ * 0.01) {
    return result;
  }

  // IPPE returns both planar solutions. Refine each and compare pixel residuals.
  std::vector<cv::Vec3d> rotations;
  std::vector<cv::Vec3d> translations;
  cv::solvePnPGeneric(points, pixels, camera, distortion, rotations, translations, false,
    cv::SOLVEPNP_IPPE);

  constexpr double max_rms_px = 3.0;
  double best_error = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < rotations.size(); ++i) {
    auto r = rotations[i];
    auto t = translations[i];
    if (!cv::checkRange(cv::Mat(r)) || !cv::checkRange(cv::Mat(t))) {
      continue;
    }
    cv::solvePnPRefineLM(points, pixels, camera, distortion, r, t);
    cv::Matx33d rotation;
    cv::Rodrigues(r, rotation);
    if (!cv::checkRange(cv::Mat(rotation)) || !cv::checkRange(cv::Mat(t)) || t[2] <= 0 ||
      std::any_of(points.begin(), points.end(), [&](const auto &p) {
        return rotation(2, 0) * p.x + rotation(2, 1) * p.y + t[2] <= 0;
      }))
    {
      continue;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(points, r, t, camera, distortion, projected);
    const double error = cv::norm(pixels, projected, cv::NORM_L2) / std::sqrt(points.size());
    if (error <= max_rms_px && error < best_error) {
      best_error = error;
      result.pose = Pose{r, t};
    }
  }
  return result;
}

void BoardDetector::draw_axes(cv::Mat &image, const Pose &pose, const cv::Mat &camera,
  const cv::Mat &distortion) const
{
  const std::vector<cv::Point3f> axes{
    {0, 0, 0}, {axis_length_, 0, 0}, {0, axis_length_, 0}, {0, 0, axis_length_}};
  std::vector<cv::Point2f> screen;
  cv::projectPoints(axes, pose.rotation, pose.translation, camera, distortion, screen);
  const cv::Scalar colors[]{{0, 0, 255}, {0, 255, 0}, {255, 0, 0}};
  for (int axis = 0; axis < 3; ++axis) {
    cv::arrowedLine(image, screen[0], screen[axis + 1], colors[axis], 3, cv::LINE_AA);
  }
}
}  // namespace board_recognition
