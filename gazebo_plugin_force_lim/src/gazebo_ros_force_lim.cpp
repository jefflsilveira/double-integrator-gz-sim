// Copyright 2013 Open Source Robotics Foundation, Inc.
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

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>

#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>


#include "gazebo_plugin_force_lim/gazebo_ros_force_lim.hpp"

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosForcePrivate
{
public:
  /// A pointer to the Link, where force is applied
  gazebo::physics::LinkPtr link_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Wrench subscriber
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;

  /// Container for the wrench force that this plugin exerts on the body.
  geometry_msgs::msg::Wrench wrench_msg_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  /// Indicates that the force should be applied on the world frame instead of the link frame
  bool force_on_world_frame_;

  // Member variables for velocity thresholds
  ignition::math::Vector3d linear_velocity_pos_threshold_;
  ignition::math::Vector3d linear_velocity_neg_threshold_;
  ignition::math::Vector3d angular_velocity_pos_threshold_;
  ignition::math::Vector3d angular_velocity_neg_threshold_;
};

GazeboRosForce::GazeboRosForce()
: impl_(std::make_unique<GazeboRosForcePrivate>())
{
}

GazeboRosForce::~GazeboRosForce()
{
}

void GazeboRosForce::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  auto logger = rclcpp::get_logger("gazebo_ros_force_lim");

  // Target link
  if (!sdf->HasElement("link_name")) {
    RCLCPP_ERROR(logger, "Force plugin missing <link_name>, cannot proceed");
    return;
  }

  auto link_name = sdf->GetElement("link_name")->Get<std::string>();

  impl_->link_ = model->GetLink(link_name);
  if (!impl_->link_) {
    RCLCPP_ERROR(logger, "Link named: %s does not exist\n", link_name.c_str());
    return;
  }

  // Force frame
  if (!sdf->HasElement("force_frame")) {
    RCLCPP_INFO(
      logger, "Force plugin missing <force_frame> wasn't set,"
      "therefore it's been set as 'world'. The other option is 'link'.");
    impl_->force_on_world_frame_ = true;
  } else {
    auto force_frame = sdf->GetElement("force_frame")->Get<std::string>();
    if (force_frame == "world") {
      impl_->force_on_world_frame_ = true;
    } else if (force_frame == "link") {
      impl_->force_on_world_frame_ = false;
    } else {
      RCLCPP_ERROR(logger, "Force plugin <force_frame> can only be 'world' or 'link'");
      return;
    }
  }

  // Linear Velocity thresholds

  // Read positive linear velocity thresholds
  float linear_velocity_pos_threshold_x;
  if (sdf->HasElement("linear_velocity_pos_threshold_x")) {
    linear_velocity_pos_threshold_x= sdf->GetElement("linear_velocity_pos_threshold_x")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <linear_velocity_pos_threshold_x>, cannot proceed");
    return;
  }

  float linear_velocity_pos_threshold_y;
  if (sdf->HasElement("linear_velocity_pos_threshold_y")) {
    linear_velocity_pos_threshold_y= sdf->GetElement("linear_velocity_pos_threshold_y")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <linear_velocity_pos_threshold_y>, cannot proceed");
    return;
  }

  float linear_velocity_pos_threshold_z;
  if (sdf->HasElement("linear_velocity_pos_threshold_z")) {
    linear_velocity_pos_threshold_z= sdf->GetElement("linear_velocity_pos_threshold_z")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <linear_velocity_pos_threshold_z>, cannot proceed");
    return;
  }
  
  impl_->linear_velocity_pos_threshold_ = ignition::math::Vector3d(linear_velocity_pos_threshold_x, linear_velocity_pos_threshold_y, linear_velocity_pos_threshold_z);
  RCLCPP_INFO(
    rclcpp::get_logger("gazebo_ros_force_lim"),
    "Positive Linear Velocity threshold: [%f, %f, %f]",
    impl_->linear_velocity_pos_threshold_.X(), impl_->linear_velocity_pos_threshold_.Y(), impl_->linear_velocity_pos_threshold_.Z());

  // Read negative linear velocity thresholds
  float linear_velocity_neg_threshold_x;
  if (sdf->HasElement("linear_velocity_neg_threshold_x")) {
    linear_velocity_neg_threshold_x= sdf->GetElement("linear_velocity_neg_threshold_x")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <linear_velocity_neg_threshold_x>, cannot proceed");
    return;
  }

  float linear_velocity_neg_threshold_y;
  if (sdf->HasElement("linear_velocity_neg_threshold_y")) {
    linear_velocity_neg_threshold_y= sdf->GetElement("linear_velocity_neg_threshold_y")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <linear_velocity_neg_threshold_y>, cannot proceed");
    return;
  }

  float linear_velocity_neg_threshold_z;
  if (sdf->HasElement("linear_velocity_neg_threshold_z")) {
    linear_velocity_neg_threshold_z= sdf->GetElement("linear_velocity_neg_threshold_z")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <linear_velocity_neg_threshold_z>, cannot proceed");
    return;
  }
  
  impl_->linear_velocity_neg_threshold_ = ignition::math::Vector3d(linear_velocity_neg_threshold_x, linear_velocity_neg_threshold_y, linear_velocity_neg_threshold_z);
  RCLCPP_INFO(
    rclcpp::get_logger("gazebo_ros_force_lim"),
    "Negative Linear Velocity threshold: [%f, %f, %f]",
    impl_->linear_velocity_neg_threshold_.X(), impl_->linear_velocity_neg_threshold_.Y(), impl_->linear_velocity_neg_threshold_.Z());


// Angular Velocity thresholds

  // Read positive angular velocity thresholds
  float angular_velocity_pos_threshold_x;
  if (sdf->HasElement("angular_velocity_pos_threshold_x")) {
    angular_velocity_pos_threshold_x= sdf->GetElement("angular_velocity_pos_threshold_x")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <angular_velocity_pos_threshold_x>, cannot proceed");
    return;
  }

  float angular_velocity_pos_threshold_y;
  if (sdf->HasElement("angular_velocity_pos_threshold_y")) {
    angular_velocity_pos_threshold_y= sdf->GetElement("angular_velocity_pos_threshold_y")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <angular_velocity_pos_threshold_y>, cannot proceed");
    return;
  }

  float angular_velocity_pos_threshold_z;
  if (sdf->HasElement("angular_velocity_pos_threshold_z")) {
    angular_velocity_pos_threshold_z= sdf->GetElement("angular_velocity_pos_threshold_z")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <angular_velocity_pos_threshold_z>, cannot proceed");
    return;
  }
  
  impl_->angular_velocity_pos_threshold_ = ignition::math::Vector3d(angular_velocity_pos_threshold_x, angular_velocity_pos_threshold_y, angular_velocity_pos_threshold_z);
  RCLCPP_INFO(
    rclcpp::get_logger("gazebo_ros_force_lim"),
    "Positive angular Velocity threshold: [%f, %f, %f]",
    impl_->angular_velocity_pos_threshold_.X(), impl_->angular_velocity_pos_threshold_.Y(), impl_->angular_velocity_pos_threshold_.Z());

  // Read negative angular velocity thresholds
  float angular_velocity_neg_threshold_x;
  if (sdf->HasElement("angular_velocity_neg_threshold_x")) {
    angular_velocity_neg_threshold_x= sdf->GetElement("angular_velocity_neg_threshold_x")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <angular_velocity_neg_threshold_x>, cannot proceed");
    return;
  }

  float angular_velocity_neg_threshold_y;
  if (sdf->HasElement("angular_velocity_neg_threshold_y")) {
    angular_velocity_neg_threshold_y= sdf->GetElement("angular_velocity_neg_threshold_y")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <angular_velocity_neg_threshold_y>, cannot proceed");
    return;
  }

  float angular_velocity_neg_threshold_z;
  if (sdf->HasElement("angular_velocity_neg_threshold_z")) {
    angular_velocity_neg_threshold_z= sdf->GetElement("angular_velocity_neg_threshold_z")->Get<float>();
  } else {
    RCLCPP_ERROR(logger, "Force plugin missing <angular_velocity_neg_threshold_z>, cannot proceed");
    return;
  }
  
  impl_->angular_velocity_neg_threshold_ = ignition::math::Vector3d(angular_velocity_neg_threshold_x, angular_velocity_neg_threshold_y, angular_velocity_neg_threshold_z);
  RCLCPP_INFO(
    rclcpp::get_logger("gazebo_ros_force_lim"),
    "Negative angular Velocity threshold: [%f, %f, %f]",
    impl_->angular_velocity_neg_threshold_.X(), impl_->angular_velocity_neg_threshold_.Y(), impl_->angular_velocity_neg_threshold_.Z());






  // Subscribe to wrench messages
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->wrench_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Wrench>(
    "gazebo_ros_force_lim", qos.get_subscription_qos("gazebo_ros_force_lim", rclcpp::SystemDefaultsQoS()),
    std::bind(&GazeboRosForce::OnRosWrenchMsg, this, std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosForce::OnUpdate, this));
}

void GazeboRosForce::OnRosWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  impl_->wrench_msg_.force.x = msg->force.x;
  impl_->wrench_msg_.force.y = msg->force.y;
  impl_->wrench_msg_.force.z = msg->force.z;
  impl_->wrench_msg_.torque.x = msg->torque.x;
  impl_->wrench_msg_.torque.y = msg->torque.y;
  impl_->wrench_msg_.torque.z = msg->torque.z;
}

void GazeboRosForce::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosForce::OnUpdate");
  IGN_PROFILE_BEGIN("Aplly forces");
#endif

  // Get the model's linear velocity
  ignition::math::Vector3d linear_velocity = impl_->link_->WorldLinearVel();

  // Get the model's angular velocity
  ignition::math::Vector3d angular_velocity = impl_->link_->WorldAngularVel();

  // Print the linear velocity
  // RCLCPP_INFO(
  //   rclcpp::get_logger("gazebo_ros_force"),
  //   "Linear Velocity: [%f, %f, %f]",
  //   linear_velocity.X(), linear_velocity.Y(), linear_velocity.Z());


  // Check each component of linear velocity and apply zero force if threshold exceeded
  if (linear_velocity.X() > impl_->linear_velocity_pos_threshold_.X() && impl_->wrench_msg_.force.x > 0.0) {
    impl_->wrench_msg_.force.x = 0.0;
  }
  if (linear_velocity.Y() > impl_->linear_velocity_pos_threshold_.Y() && impl_->wrench_msg_.force.y > 0.0) {
    impl_->wrench_msg_.force.y = 0.0;
  }
  if (linear_velocity.Z() > impl_->linear_velocity_pos_threshold_.Z() && impl_->wrench_msg_.force.z > 0.0) {
    impl_->wrench_msg_.force.z = 0.0;
  }


  if (linear_velocity.X() < impl_->linear_velocity_neg_threshold_.X() && impl_->wrench_msg_.force.x < 0.0) {
    impl_->wrench_msg_.force.x = 0.0;
  }
  if (linear_velocity.Y() < impl_->linear_velocity_neg_threshold_.Y() && impl_->wrench_msg_.force.y < 0.0) {
    impl_->wrench_msg_.force.y = 0.0;
  }
  if (linear_velocity.Z() < impl_->linear_velocity_neg_threshold_.Z() && impl_->wrench_msg_.force.z < 0.0) {
    impl_->wrench_msg_.force.z = 0.0;
  }


  // // Check each component of angular velocity and apply zero torque if threshold exceeded
  // if (angular_velocity.X() > impl_->angular_velocity_pos_threshold_.X() && impl_->wrench_msg_.torque.x > 0.0) {
  //   impl_->wrench_msg_.torque.x = 0.0;
  // }
  // if (angular_velocity.Y() > impl_->angular_velocity_pos_threshold_.Y() && impl_->wrench_msg_.torque.y > 0.0) {
  //   impl_->wrench_msg_.torque.y = 0.0;
  // }
  // if (angular_velocity.Z() > impl_->angular_velocity_pos_threshold_.Z() && impl_->wrench_msg_.torque.z > 0.0) {
  //   impl_->wrench_msg_.torque.z = 0.0;
  // }

  // if (angular_velocity.X() < impl_->angular_velocity_neg_threshold_.X() && impl_->wrench_msg_.torque.x < 0.0) {
  //   impl_->wrench_msg_.torque.x = 0.0;
  // }
  // if (angular_velocity.Y() < impl_->angular_velocity_neg_threshold_.Y() && impl_->wrench_msg_.torque.y < 0.0) {
  //   impl_->wrench_msg_.torque.y = 0.0;
  // }
  // if (angular_velocity.Z() < impl_->angular_velocity_neg_threshold_.Z() && impl_->wrench_msg_.torque.z < 0.0) {
  //   impl_->wrench_msg_.torque.z = 0.0;
  // }




  if (impl_->force_on_world_frame_) {
    impl_->link_->AddForce(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.force));
    impl_->link_->AddTorque(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.torque));
  } else {
    impl_->link_->AddRelativeForce(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.force));
    impl_->link_->AddRelativeTorque(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.torque));
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce)
}  // namespace gazebo_plugins
