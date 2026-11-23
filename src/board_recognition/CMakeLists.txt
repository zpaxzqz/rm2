cmake_minimum_required(VERSION 3.8)
project(board_recognition)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV 4.5 REQUIRED COMPONENTS aruco calib3d core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(board_recognition_node src/recognition_node.cpp src/board_detector.cpp)
target_compile_features(board_recognition_node PRIVATE cxx_std_17)
target_compile_options(board_recognition_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(board_recognition_node ${OpenCV_LIBS})
ament_target_dependencies(board_recognition_node
  ament_index_cpp cv_bridge geometry_msgs interfaces rclcpp sensor_msgs tf2)

install(TARGETS board_recognition_node DESTINATION lib/${PROJECT_NAME})
install(FILES ../../config/board.yaml ../../config/DA3113952.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
