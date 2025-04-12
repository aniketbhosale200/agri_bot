// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#define PLANAR_POINT_DIM 3

namespace mecanum_drive_controller
{
class Odometry
{
public:
  typedef std::function<void(double, double, double)> IntegrationFunction;

  Odometry();

  void init(const rclcpp::Time & time, std::array<double, PLANAR_POINT_DIM> base_frame_offset);

  bool update(
    const double wheel_front_left_pos, const double wheel_front_left_vel,
    const double wheel_rear_left_pos, const double wheel_rear_left_vel,
    const double wheel_rear_right_pos, const double wheel_rear_right_vel,
    const double wheel_front_right_pos, const double wheel_front_right_vel,
    const double dt);

  double getX() const { return position_x_in_base_frame_; }
  double getY() const { return position_y_in_base_frame_; }
  double getRz() const { return orientation_z_in_base_frame_; }
  double getVx() const { return velocity_in_base_frame_linear_x; }
  double getVy() const { return velocity_in_base_frame_linear_y; }
  double getWz() const { return velocity_in_base_frame_angular_z; }

  const std::array<double, PLANAR_POINT_DIM> & getBaseFrameOffset() const
  {
    return base_frame_offset_;
  }

  void setWheelsParams(
    const double sum_of_robot_center_projection_on_X_Y_axis,
    const double circular_radius,
    const double semi_major_axis,
    const double semi_minor_axis);

private:
  rclcpp::Time timestamp_;
  std::array<double, PLANAR_POINT_DIM> base_frame_offset_;
  double position_x_in_base_frame_;
  double position_y_in_base_frame_;
  double orientation_z_in_base_frame_;
  double velocity_in_base_frame_linear_x;
  double velocity_in_base_frame_linear_y;
  double velocity_in_base_frame_angular_z;

  double sum_of_robot_center_projection_on_X_Y_axis_;
  double circular_radius_;
  double semi_major_axis_;
  double semi_minor_axis_;

  double compute_effective_radius(double theta) const;
};

}  // namespace mecanum_drive_controller

#endif  // MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_
