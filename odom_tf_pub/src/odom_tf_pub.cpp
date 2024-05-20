#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

class FramePublisher : public rclcpp::Node
{
  public:
    FramePublisher()
    : Node("odom_frame_publisher")
    {

      // Initialize the transform broadcaster
      tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Subscribe to a turtle{1}{2}/pose topic and call odom_callback
      // callback function on each message
      std::string topic_name = "/odom";

      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name, 10,
        std::bind(&FramePublisher::odom_callback, this, std::placeholders::_1));
    }

  private:
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
      geometry_msgs::msg::TransformStamped t;

      // Read message content and assign it to
      // corresponding tf variables
      t.header.stamp = this->now();
      t.header.frame_id = msg->header.frame_id; // Parent frame from the odometry message
      t.child_frame_id = msg->child_frame_id;   // Child frame from the odometry message

      t.transform.translation.x = msg->pose.pose.position.x;
      t.transform.translation.y = msg->pose.pose.position.y;
      t.transform.translation.z = msg->pose.pose.position.z;


      t.transform.rotation.x = msg->pose.pose.orientation.x;
      t.transform.rotation.y = msg->pose.pose.orientation.y;
      t.transform.rotation.z = msg->pose.pose.orientation.z;
      t.transform.rotation.w = msg->pose.pose.orientation.w;

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}