/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

 #include <std_msgs/msg/int8.hpp>
 #include <geometry_msgs/msg/transform_stamped.hpp>
 #include <geometry_msgs/msg/pose.hpp>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 #include <tf2/LinearMath/Vector3.h>
 #include <tf2/LinearMath/Transform.h> 
 
 #include <moveit_servo/servo.h>
 #include <moveit_servo/pose_tracking.h>
 #include <moveit_servo/status_codes.h>
 #include <moveit_servo/servo_parameters.h>
 #include <moveit_servo/servo_parameters.h>
 #include <moveit_servo/make_shared_from_pool.h>
 #include <thread>
 #include <mutex>  // For std::mutex
 #include <Eigen/Geometry>
 
 static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking_demo");
 
 // Class for monitoring status of moveit_servo
 class StatusMonitor
 {
 public:
   StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
   {
     sub_ = node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
                                                           [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) {
                                                             return statusCB(msg);
                                                           });
   }
 
 private:
   void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
   {
     moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
     if (latest_status != status_)
     {
       status_ = latest_status;
       const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
       RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
     }
   }
 
   moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
   rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
 };
 
 // Function to use the camera orientation values
 geometry_msgs::msg::Quaternion QuaternionRotation(const geometry_msgs::msg::Quaternion& q1,
                                                  const geometry_msgs::msg::Quaternion& q2){
  
    tf2::Quaternion current_quat;
    tf2::fromMsg(q1, current_quat);

    tf2::Quaternion delta_quat;
    tf2::fromMsg(q2, delta_quat);

    auto new_quat = current_quat*delta_quat;
    return tf2::toMsg(new_quat.normalize());
 }

 tf2::Vector3 TransformToGlobal(const geometry_msgs::msg::Pose& camera_delta, const geometry_msgs::msg::TransformStamped& current_ee_tf)
 {
    const tf2::Vector3 local_positions(
        static_cast<tf2Scalar>(camera_delta.position.x),
        static_cast<tf2Scalar>(-camera_delta.position.y),
        static_cast<tf2Scalar>(camera_delta.position.z)
    );

    const tf2::Quaternion quat(
        static_cast<tf2Scalar>(current_ee_tf.transform.rotation.x),
        static_cast<tf2Scalar>(current_ee_tf.transform.rotation.y),
        static_cast<tf2Scalar>(current_ee_tf.transform.rotation.z),
        static_cast<tf2Scalar>(current_ee_tf.transform.rotation.w)
    );

    tf2::Vector3 world_postion = tf2::quatRotate(quat, local_positions);
    return world_postion;
 }


 /**
  * Instantiate the pose tracking interface.
  * Subscribe to camera_target_pose topic and use it to update the target pose
  */
 int main(int argc, char** argv)
 {
   rclcpp::init(argc, argv);
   rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pose_tracking");
 
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node);
   std::thread executor_thread([&executor]() { executor.spin(); });
 
   auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
 
   if (servo_parameters == nullptr)
   {
     RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
     exit(EXIT_FAILURE);
   }
 
   // Load the planning scene monitor
   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
   planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
   if (!planning_scene_monitor->getPlanningScene())
   {
     RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
     exit(EXIT_FAILURE);
   }
 
   planning_scene_monitor->providePlanningSceneService();
   planning_scene_monitor->startSceneMonitor();
   planning_scene_monitor->startWorldGeometryMonitor(
       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
       false /* skip octomap monitor */);
   planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
   planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
 
   // Wait for Planning Scene Monitor to setup
   if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 10.0 /* seconds */))
   {
     RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
     exit(EXIT_FAILURE);
   }
 
   // Create the pose tracker
   moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);
 
   // Make a publisher for sending pose commands
   auto target_pose_pub =
       node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());
 
   // Subscribe to servo status (and log it when it changes)
   StatusMonitor status_monitor(node, servo_parameters->status_topic);
 
   Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
   double rot_tol = 0.01;
 
   // Get the current EE transform
   geometry_msgs::msg::TransformStamped current_ee_tf;
   tracker.getCommandFrameTransform(current_ee_tf);
 
   // Convert it to a Pose
   geometry_msgs::msg::PoseStamped target_pose;
   target_pose.header.frame_id = current_ee_tf.header.frame_id;
   target_pose.pose.position.x = current_ee_tf.transform.translation.x;
   target_pose.pose.position.y = current_ee_tf.transform.translation.y;
   target_pose.pose.position.z = current_ee_tf.transform.translation.z;
   target_pose.pose.orientation = current_ee_tf.transform.rotation;
 
   // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
   // waypoints
   tracker.resetTargetPose();
 
   // Publish target pose
   target_pose.header.stamp = node->now();
   target_pose_pub->publish(target_pose);
 
   // Create shared data for camera_target_pose subscriber
   std::mutex target_pose_mutex;
   geometry_msgs::msg::PoseStamped latest_camera_target_pose = target_pose;  // Initialize with current target_pose
   bool new_camera_pose_received = false;

   std::cout <<"First Target pose " << target_pose.pose.position.z << std::endl;
 
   // Subscribe to camera_target_pose
   auto camera_target_sub = node->create_subscription<geometry_msgs::msg::Pose>(
       "camera_target_pose", rclcpp::SystemDefaultsQoS(),
       [&](const geometry_msgs::msg::Pose::ConstSharedPtr msg) {
         std::lock_guard<std::mutex> lock(target_pose_mutex);
         tracker.getCommandFrameTransform(current_ee_tf);
         auto delta_position = TransformToGlobal(*msg, current_ee_tf);

         latest_camera_target_pose.header.stamp = node->now();
         latest_camera_target_pose.pose.position.x = current_ee_tf.transform.translation.x + (delta_position.x())*(0.5);
         latest_camera_target_pose.pose.position.y = current_ee_tf.transform.translation.y + (delta_position.y())*(0.5);
         latest_camera_target_pose.pose.position.z = current_ee_tf.transform.translation.z + (delta_position.z())*(0.5);
         latest_camera_target_pose.pose.orientation = QuaternionRotation(current_ee_tf.transform.rotation,msg->orientation);
         //std::cout << "Positions: " << current_ee_tf.transform.translation.x << " " << current_ee_tf.transform.translation.y
         //           << "  " << current_ee_tf.transform.translation.z << std::endl;
         new_camera_pose_received = true;
       });
 
   // Run the pose tracking in a new thread
   std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
    while (rclcpp::ok()) {  
      moveit_servo::PoseTrackingStatusCode tracking_status =
          tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
      //RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
      //                               << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
    }
    });
 
   rclcpp::WallRate loop_rate(50);
   while (rclcpp::ok() && new_camera_pose_received == false)
   {
     loop_rate.sleep();  // Wait for first camera pose
   }
 
   while(rclcpp::ok())
   {
     // Update target_pose from camera if new data is available
     {
       std::lock_guard<std::mutex> lock(target_pose_mutex);
       if (new_camera_pose_received)
       {
         target_pose = latest_camera_target_pose;
         new_camera_pose_received = false;
       }
     }
     
     // Publish the updated target pose
     target_pose.header.stamp = node->now();
     target_pose_pub->publish(target_pose);
 
     loop_rate.sleep();
   }
 
   // Make sure the tracker is stopped and clean up
   move_to_pose_thread.join();
 
   // Kill executor thread before shutdown
   executor.cancel();
   executor_thread.join();
 
   rclcpp::shutdown();
   return EXIT_SUCCESS;
 }