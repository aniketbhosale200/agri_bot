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

#include "mecanum_drive_controller/odometry.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mecanum_drive_controller
{
Odometry::Odometry()
: timestamp_(0.0),
  position_x_in_base_frame_(0.0),
  position_y_in_base_frame_(0.0),
  orientation_z_in_base_frame_(0.0),
  velocity_in_base_frame_linear_x(0.0),
  velocity_in_base_frame_linear_y(0.0),
  velocity_in_base_frame_angular_z(0.0),
  sum_of_robot_center_projection_on_X_Y_axis_(0.0),
  circular_radius_(0.0),
  semi_major_axis_(0.0),
  semi_minor_axis_(0.0)
{
}

void Odometry::init(
  const rclcpp::Time & time, std::array<double, PLANAR_POINT_DIM> base_frame_offset)
{
  timestamp_ = time;
  base_frame_offset_[0] = base_frame_offset[0];
  base_frame_offset_[1] = base_frame_offset[1];
  base_frame_offset_[2] = base_frame_offset[2];
}

bool Odometry::update(
  const double wheel_front_left_pos, const double wheel_front_left_vel,
  const double wheel_rear_left_pos, const double wheel_rear_left_vel,
  const double wheel_rear_right_pos, const double wheel_rear_right_vel,
  const double wheel_front_right_pos, const double wheel_front_right_vel,
  const double dt)
{
  if (dt < 0.0001) return false;  // Interval too small to integrate with

  // Compute effective radii based on wheel positions
  const double r_fl = compute_effective_radius(wheel_front_left_pos);
  const double r_fr = compute_effective_radius(wheel_front_right_pos);
  const double r_rr = compute_effective_radius(wheel_rear_right_pos);
  const double r_rl = compute_effective_radius(wheel_rear_left_pos);

  // Forward kinematics with dynamic radii
  double velocity_in_center_frame_linear_x =
    0.25 * (r_fl * wheel_front_left_vel + r_rl * wheel_rear_left_vel +
            r_rr * wheel_rear_right_vel + r_fr * wheel_front_right_vel);
  double velocity_in_center_frame_linear_y =
    0.25 * (-r_fl * wheel_front_left_vel + r_rl * wheel_rear_left_vel -
            r_rr * wheel_rear_right_vel + r_fr * wheel_front_right_vel);
  double velocity_in_center_frame_angular_z =
    0.25 * (-r_fl * wheel_front_left_vel - r_rl * wheel_rear_left_vel +
            r_rr * wheel_rear_right_vel + r_fr * wheel_front_right_vel) /
    sum_of_robot_center_projection_on_X_Y_axis_;

  tf2::Quaternion orientation_R_c_b;
  orientation_R_c_b.setRPY(0.0, 0.0, -base_frame_offset_[2]);

  tf2::Matrix3x3 angular_transformation_from_center_2_base = tf2::Matrix3x3(orientation_R_c_b);
  tf2::Vector3 velocity_in_center_frame_w_r_t_base_frame_ =
    angular_transformation_from_center_2_base *
    tf2::Vector3(velocity_in_center_frame_linear_x, velocity_in_center_frame_linear_y, 0.0);
  tf2::Vector3 linear_transformation_from_center_2_base =
    angular_transformation_from_center_2_base *
    tf2::Vector3(-base_frame_offset_[0], -base_frame_offset_[1], 0.0);

  velocity_in_base_frame_linear_x =
    velocity_in_center_frame_w_r_t_base_frame_.x() +
    linear_transformation_from_center_2_base.y() * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_linear_y =
    velocity_in_center_frame_w_r_t_base_frame_.y() -
    linear_transformation_from_center_2_base.x() * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_angular_z = velocity_in_center_frame_angular_z;

  // Integration
  orientation_z_in_base_frame_ += velocity_in_base_frame_angular_z * dt;

  tf2::Quaternion orientation_R_b_odom;
  orientation_R_b_odom.setRPY(0.0, 0.0, orientation_z_in_base_frame_);

  tf2::Matrix3x3 angular_transformation_from_base_2_odom = tf2::Matrix3x3(orientation_R_b_odom);
  tf2::Vector3 velocity_in_base_frame_w_r_t_odom_frame_ =
    angular_transformation_from_base_2_odom *
    tf2::Vector3(velocity_in_base_frame_linear_x, velocity_in_base_frame_linear_y, 0.0);

  position_x_in_base_frame_ += velocity_in_base_frame_w_r_t_odom_frame_.x() * dt;
  position_y_in_base_frame_ += velocity_in_base_frame_w_r_t_odom_frame_.y() * dt;

  return true;
}

void Odometry::setWheelsParams(
  const double sum_of_robot_center_projection_on_X_Y_axis,
  const double circular_radius,
  const double semi_major_axis,
  const double semi_minor_axis)
{
  sum_of_robot_center_projection_on_X_Y_axis_ = sum_of_robot_center_projection_on_X_Y_axis;
  circular_radius_ = circular_radius;
  semi_major_axis_ = semi_major_axis;
  semi_minor_axis_ = semi_minor_axis;
}

double Odometry::compute_effective_radius(double theta) const
{
  // Normalize theta to [0, 2π)
  theta = std::fmod(theta, 2 * M_PI);
  if (theta < 0) theta += 2 * M_PI;

  // Circular part: 0 to π, Elliptical part: π to 2π
  if (theta < M_PI)
  {
    return circular_radius_;  // Constant radius for circular half
  }
  else
  {
    double phi = theta - M_PI;  // Adjust angle for elliptical part
    return (semi_major_axis_ * semi_minor_axis_) /
           std::sqrt(
             std::pow(semi_minor_axis_ * std::cos(phi), 2) +
             std::pow(semi_major_axis_ * std::sin(phi), 2));  // Ellipse radius formula
  }
}

}  // namespace mecanum_drive_controller
