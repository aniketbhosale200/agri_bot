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

#ifndef MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/msg/mecanum_drive_controller_state.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "mecanum_drive_controller/mecanum_drive_controller_parameters.hpp"
#include "mecanum_drive_controller/odometry.hpp"
#include "mecanum_drive_controller/visibility_control.h"

namespace mecanum_drive_controller
{
static constexpr size_t NR_STATE_ITFS = 4;
static constexpr size_t NR_CMD_ITFS = 4;
static constexpr size_t NR_REF_ITFS = 3;

class MecanumDriveController : public controller_interface::ChainableControllerInterface
{
public:
  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  MecanumDriveController();

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  MECANUM_DRIVE_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerReferenceMsgUnstamped = geometry_msgs::msg::Twist;
  using OdomStateMsg = nav_msgs::msg::Odometry;
  using TfStateMsg = tf2_msgs::msg::TFMessage;
  using ControllerStateMsg = control_msgs::msg::MecanumDriveControllerState;

protected:
  std::shared_ptr<mecanum_drive_controller::ParamListener> param_listener_;
  mecanum_drive_controller::Params params_;

  // Geometry parameters for semi-circular wheels
  double circular_radius_;      // Radius of circular part
  double semi_major_axis_;      // Semi-major axis of elliptical part
  double semi_minor_axis_;      // Semi-minor axis of elliptical part

  enum WheelIndex : std::size_t
  {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_RIGHT = 2,
    REAR_LEFT = 3
  };

  std::vector<std::string> command_joint_names_;
  std::vector<std::string> state_joint_names_;
  std::vector<std::string> reference_names_;

  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerReferenceMsgUnstamped>::SharedPtr ref_unstamped_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);

  using OdomStatePublisher = realtime_tools::RealtimePublisher<OdomStateMsg>;
  rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_s_publisher_;
  std::unique_ptr<OdomStatePublisher> rt_odom_state_publisher_;

  using TfStatePublisher = realtime_tools::RealtimePublisher<TfStateMsg>;
  rclcpp::Publisher<TfStateMsg>::SharedPtr tf_odom_s_publisher_;
  std::unique_ptr<TfStatePublisher> rt_tf_odom_state_publisher_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

  Odometry odometry_;
  bool use_stamped_vel_ = true;

private:
  MECANUM_DRIVE_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  MECANUM_DRIVE_CONTROLLER__VISIBILITY_LOCAL
  void reference_unstamped_callback(const std::shared_ptr<ControllerReferenceMsgUnstamped> msg);

  double compute_effective_radius(double theta) const;

  double velocity_in_center_frame_linear_x_;   // [m/s]
  double velocity_in_center_frame_linear_y_;   // [m/s]
  double velocity_in_center_frame_angular_z_;  // [rad/s]
};

}  // namespace mecanum_drive_controller

#endif  // MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
