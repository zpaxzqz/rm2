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
namespace
{
// On even-height boards the two OpenCV patterns stagger opposite rows.
void use_modern_pattern(cv::aruco::CharucoBoard &board, int height, float square)
{
  if (height % 2 != 0) {
    return;
  }

  for (auto &marker : board.objPoints) {
    const float center_y = (marker[0].y + marker[2].y) / 2.0F;
    const int row_from_top = height - 1 - static_cast<int>(center_y / square);
    const int direction = row_from_top % 2 == 0 ? 1 : -1;
    for (auto &corner : marker) {
      corner.x += direction * square;
    }
  }
  for (size_t i = 0; i < board.chessboardCorners.size(); ++i) {
    auto &indices = board.nearestMarkerIdx[i];
    auto &corners = board.nearestMarkerCorners[i];
    indices.clear();
    corners.clear();
    const auto &corner = board.chessboardCorners[i];
    for (size_t j = 0; j < board.objPoints.size(); ++j) {
      const auto &marker = board.objPoints[j];
      const auto center = (marker[0] + marker[2]) * 0.5F;
      if (std::abs(center.x - corner.x) < square * 0.75F &&
        std::abs(center.y - corner.y) < square * 0.75F)
      {
        const auto nearest = std::min_element(marker.begin(), marker.end(),
          [&corner](const auto &a, const auto &b) {
            return cv::norm(a - corner) < cv::norm(b - corner);
          });
        indices.push_back(static_cast<int>(j));
        corners.push_back(static_cast<int>(nearest - marker.begin()));
      }
    }
  }
}
}  // namespace

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
  int legacy_pattern = 1;
  if (!config["legacy_pattern"].empty()) {
    config["legacy_pattern"] >> legacy_pattern;
  }
  if (legacy_pattern != 0 && legacy_pattern != 1) {
    throw std::runtime_error("legacy_pattern must be 0 or 1");
  }
  if (!legacy_pattern) {
    if (width % 2 != 0 && height % 2 == 0) {
      throw std::runtime_error("Modern pattern with even height requires even width");
    }
    use_modern_pattern(*board_, height, square);
  }
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
  std::vector<cv::Point3f> points;
  std::vector<cv::Point2f> plane;
  const auto add_point = [&](const cv::Point3f &corner) {
    const float x = static_cast<float>(corner.x - center_[0]);
    const float y = static_cast<float>(corner.y - center_[1]);
    points.emplace_back(x, y, 0.0F);
    plane.emplace_back(x, y);
  };
  if (corner_ids.size() >= 4) {
    for (const int id : corner_ids) {
      add_point(board_->chessboardCorners.at(id));
    }
  }
  if (points.size() < 4) {
    if (board_ids.size() < 2) {
      return result;
    }
    points.clear();
    plane.clear();
    pixels.clear();
    for (size_t i = 0; i < board_ids.size(); ++i) {
      const auto marker = std::find(board_->ids.begin(), board_->ids.end(), board_ids[i]);
      for (const auto &corner : board_->objPoints[marker - board_->ids.begin()]) {
        add_point(corner);
      }
      pixels.insert(pixels.end(), board_borders[i].begin(), board_borders[i].end());
    }
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

std::vector<cv::Point2f> BoardDetector::project_marker(int id, const Pose &pose,
  const cv::Mat &camera, const cv::Mat &distortion, cv::Size size) const
{
  const auto marker = std::find(board_->ids.begin(), board_->ids.end(), id);
  if (marker == board_->ids.end()) {
    return {};
  }

  cv::Matx33d rotation;
  cv::Rodrigues(pose.rotation, rotation);
  std::vector<cv::Point3f> points;
  for (const auto &corner : board_->objPoints[marker - board_->ids.begin()]) {
    const double x = corner.x - center_[0];
    const double y = corner.y - center_[1];
    if (rotation(2, 0) * x + rotation(2, 1) * y + pose.translation[2] <= 0) {
      return {};
    }
    points.emplace_back(x, y, 0.0);
  }

  std::vector<cv::Point2f> pixels;
  cv::projectPoints(points, pose.rotation, pose.translation, camera, distortion, pixels);
  for (const auto &p : pixels) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      p.x < 0 || p.x >= size.width || p.y < 0 || p.y >= size.height)
    {
      return {};
    }
  }
  return pixels;
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
