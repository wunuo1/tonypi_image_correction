# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(tonypi_image_correction)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hbm_img_msgs REQUIRED)
include_directories(include
  /usr/include
  /usr/include/opencv/
)

add_executable(tonypi_image_correction
  src/image_correct.cpp
)


ament_target_dependencies(
  tonypi_image_correction
    rclcpp
    sensor_msgs
    cv_bridge
    hbm_img_msgs
)
target_link_libraries(tonypi_image_correction opencv_world hbmem )

install(
  TARGETS tonypi_image_correction
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()