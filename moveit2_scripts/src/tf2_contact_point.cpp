#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class ContactPoint : public rclcpp::Node
{
public:
  explicit ContactPoint()
  : Node("contact_point_tf2_broadcaster")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
  }

private:
  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "end_effector_link";
    t.child_frame_id = "gripper_contact_point";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.13;
    tf2::Quaternion q;
    t.transform.rotation.x = 0.00;
    t.transform.rotation.y = 0.00;
    t.transform.rotation.z = 0.00;
    t.transform.rotation.w = 1.00;

    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContactPoint>());
  rclcpp::shutdown();
  return 0;
}