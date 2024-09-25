//
// Created by Martin Økter on 19/12/2023.
//
#include <chrono>
#include <iostream>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "../include/icearm_ctrl/icearm_ctrl.h"

using namespace std::chrono_literals;

class IceArmInterface : public rclcpp::Node
{
 public:
  IceArmInterface()
  : Node("arm_ctrl")
  , RobotArm()
  {
    arm_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/icearm_pos", 10);    // Finger force publisher
    arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/icearm_input", 10, std::bind(&IceArmInterface::set_goal_point_from_msg, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(10ms, std::bind(&IceArmInterface::timer_callback, this));

    RobotArm.set_goal_point(0.05, 0.05, 0.05);

    std::signal(SIGINT, &IceArmInterface::onShutdown);

    RCLCPP_INFO(this->get_logger(), "Setup completed");
  }

 private:
  void timer_callback() {
    // Check if robot reached goal
    if (RobotArm.robotGoal()) {
      if (!goal_registered_flag) {
        RCLCPP_INFO(this->get_logger(), "Reached Goal");
        goal_registered_flag = true;
      }
    } else {
      // Run the main loop of the robot
      RobotArm.robotRun();
      goal_registered_flag = false;
    }

    // Post new values
    apply_arm_pos_to_msg();
  }

  void set_goal_point_from_msg(geometry_msgs::msg::Twist::SharedPtr msg){
    // Update position
    double posX = msg->linear.x;
    double posY = msg->linear.y;
    double posZ = msg->linear.z;

    RobotArm.set_goal_point(posX, posY, posZ);

    RCLCPP_INFO(this->get_logger(), "New point received");
  }

  void apply_arm_pos_to_msg()
  {
    ArmServoPos RobotPos_1 = RobotArm.return_robot_pos();

    if (RobotArm.robotOk()){
      auto arm_msg = geometry_msgs::msg::Twist();

      arm_msg.linear.x = static_cast<int>(RobotPos_1.base);
      arm_msg.linear.y = static_cast<int>(RobotPos_1.arm);
      arm_msg.linear.z = static_cast<int>(RobotPos_1.forarm);
      arm_msg.angular.x = static_cast<int>(RobotPos_1.tool);

      arm_pub_->publish(arm_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Point out of reach");
      exit(0);
    }
  }

  static void onShutdown(int signum) {
    if (signum == SIGINT) {
      // Perform cleanup operations before shutting down
      RCLCPP_INFO(rclcpp::get_logger("arm_ctrl"), "Ice gone, meltdown begun...");
      // Add your cleanup logic here

      // Call the ROS 2 shutdown function
      rclcpp::shutdown();
    }
  }

  // ROS2 and publishers declarations
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr arm_sub_;

  IceArmCtrl RobotArm;

  bool goal_registered_flag = false;
};

std::atomic<bool> shutdown_requested(false);

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IceArmInterface>();

  // Create a SingleThreadedExecutor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Add your node to the executor
  executor.add_node(node);

  // Run the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}