#include "board_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/board_observation.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

namespace board_recognition
{
namespace
{
constexpr char optical_frame[] = "camera_frame";
constexpr char window_name[] = "ChArUco recognition";
using SteadyClock = std::chrono::steady_clock;
using Observation = interfaces::msg::BoardObservation;

geometry_msgs::msg::Pose to_message(const Pose &pose)
{
  cv::Matx33d rotation;
  cv::Rodrigues(pose.rotation, rotation);
  tf2::Quaternion quaternion;
  tf2::Matrix3x3(
    rotation(0, 0), rotation(0, 1), rotation(0, 2),
    rotation(1, 0), rotation(1, 1), rotation(1, 2),
    rotation(2, 0), rotation(2, 1), rotation(2, 2)).getRotation(quaternion);
  quaternion.normalize();

  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation[0];
  msg.position.y = pose.translation[1];
  msg.position.z = pose.translation[2];
  msg.orientation.x = quaternion.x();
  msg.orientation.y = quaternion.y();
  msg.orientation.z = quaternion.z();
  msg.orientation.w = quaternion.w();
  return msg;
}

void add_status_text(cv::Mat &image, const Observation &observation)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(2);
  if (observation.has_last_pose) {
    const auto &p = observation.last_pose.position;
    if (!observation.pose_valid) {
      text << "STALE " << observation.pose_age_sec << " s | ";
    }
    text << "XYZ (m): " << p.x << ", " << p.y << ", " << p.z;
  } else {
    text << "NO POSE";
  }
  const cv::Point origin(16, 32);
  cv::putText(image, text.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 0}, 4, cv::LINE_AA);
  cv::putText(image, text.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.7,
    observation.pose_valid ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 210, 255),
    1, cv::LINE_AA);
}
}  // namespace

class RecognitionNode : public rclcpp::Node
{
public:
  RecognitionNode()
  : Node("board_recognition"),
    detector_(declare_parameter<std::string>("board_config",
      ament_index_cpp::get_package_share_directory("board_recognition") + "/config/board.yaml"))
  {
    show_window_ = declare_parameter("show_window", false);
    const double seconds = declare_parameter("input_timeout_sec", 0.5);
    if (!std::isfinite(seconds) || seconds <= 0) {
      throw std::invalid_argument("input_timeout_sec must be positive and finite.");
    }
    input_timeout_ = std::chrono::duration<double>(seconds);
    if (show_window_ && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
      show_window_ = false;
      RCLCPP_WARN(get_logger(), "No display available. Debug images will still be published.");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/recognition/board_pose", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/recognition/debug_image", sensor_qos);
    observation_pub_ = create_publisher<Observation>("/recognition/board_observation", 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/front/camera_info", sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { camera_info_ = msg; });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/front/capture", sensor_qos,
      std::bind(&RecognitionNode::on_image, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (SteadyClock::now() - last_input_ >= input_timeout_) {
        Observation observation;
        observation.header.stamp = now();
        observation.header.frame_id = optical_frame;
        observation.input_stale = true;
        fill_last_pose(observation);
        observation_pub_->publish(observation);
        if (show_window_ && !last_debug_.empty()) {
          cv::Mat preview = last_debug_.clone();
          add_status_text(preview, observation);
          show_preview(preview);
        }
      }
    });
  }

private:
  void load_calibration(const sensor_msgs::msg::Image &image, cv::Mat &camera,
    cv::Mat &distortion) const
  {
    if (!camera_info_ || camera_info_->width != image.width || camera_info_->height != image.height) {
      return;
    }
    const auto &info = *camera_info_;
    if ((!info.header.frame_id.empty() && !image.header.frame_id.empty() &&
      info.header.frame_id != image.header.frame_id) ||
      info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset || info.roi.y_offset ||
      info.roi.width || info.roi.height ||
      info.k[0] <= 0 || info.k[4] <= 0 || info.k[8] != 1.0 ||
      !std::all_of(info.k.begin(), info.k.end(), [](double x) { return std::isfinite(x); }) ||
      !std::all_of(info.d.begin(), info.d.end(), [](double x) { return std::isfinite(x); }))
    {
      return;
    }
    if ((info.distortion_model == "plumb_bob" && info.d.size() == 5) ||
      (info.distortion_model == "rational_polynomial" && info.d.size() == 8))
    {
      distortion = cv::Mat(info.d, true);
    } else if (!info.d.empty() ||
      (!info.distortion_model.empty() && info.distortion_model != "plumb_bob" &&
      info.distortion_model != "rational_polynomial"))
    {
      return;
    }
    camera = cv::Mat(3, 3, CV_64F);
    std::copy(info.k.begin(), info.k.end(), camera.ptr<double>());
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr image)
  {
    last_input_ = SteadyClock::now();
    Observation observation;
    observation.header.stamp = image->header.stamp;
    observation.header.frame_id = optical_frame;
    cv::Mat debug;
    try {
      debug = cv_bridge::toCvCopy(image, "bgr8")->image;
      cv::Mat camera, distortion;
      load_calibration(*image, camera, distortion);
      if (camera.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Waiting for compatible full-frame pinhole CameraInfo.");
      }

      const auto result = detector_.detect(debug, camera, distortion);
      observation.tag_ids.assign(result.ids.begin(), result.ids.end());
      if (!result.ids.empty()) {
        cv::aruco::drawDetectedMarkers(debug, result.borders, result.ids);
      }
      if (result.pose) {
        detector_.draw_axes(debug, *result.pose, camera, distortion);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = observation.header;
        pose.pose = to_message(*result.pose);
        last_pose_ = pose;
        observation.pose_valid = true;
        pose_pub_->publish(pose);
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Frame error: %s", error.what());
    }
    fill_last_pose(observation);
    observation_pub_->publish(observation);
    if (!debug.empty()) {
      if (show_window_) {
        last_debug_ = debug.clone();
      }
      add_status_text(debug, observation);
      debug_pub_->publish(*cv_bridge::CvImage(observation.header, "bgr8", debug).toImageMsg());
      show_preview(debug);
    }
  }

  void fill_last_pose(Observation &observation)
  {
    observation.has_last_pose = last_pose_.has_value();
    observation.pose_age_sec = -1.0;
    if (last_pose_) {
      observation.last_pose = last_pose_->pose;
      observation.last_pose_stamp = last_pose_->header.stamp;
      const auto saved = rclcpp::Time(observation.last_pose_stamp, get_clock()->get_clock_type());
      observation.pose_age_sec = std::max(0.0, (now() - saved).seconds());
    }
  }

  void show_preview(const cv::Mat &image)
  {
    if (!show_window_) {
      return;
    }
    try {
      cv::imshow(window_name, image);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE) < 1) {
        show_window_ = false;
        cv::destroyWindow(window_name);
      }
    } catch (const cv::Exception &error) {
      show_window_ = false;
      RCLCPP_WARN(get_logger(), "Debug window closed: %s", error.what());
    }
  }

  BoardDetector detector_;
  bool show_window_ = false;
  cv::Mat last_debug_;
  std::optional<geometry_msgs::msg::PoseStamped> last_pose_;
  std::chrono::duration<double> input_timeout_;
  SteadyClock::time_point last_input_ = SteadyClock::now();
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<Observation>::SharedPtr observation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace board_recognition

int main(int argc, char **argv)
{
  // Limit internal OpenCV workers so they do not contend with ROS and video decoding.
  cv::setNumThreads(2);
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<board_recognition::RecognitionNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("board_recognition"), "%s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
