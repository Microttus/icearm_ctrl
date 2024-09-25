//
// Created by Martin Økter on 19/12/2023.
//
#include <chrono>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "../include/icearm_ctrl/ice_serial_servo.h"

using namespace std::chrono_literals;

struct ArmServoPos{
  int base = 0;
  int arm = 0;
  int forarm = 0;
  int tool = 0;
};

struct ToolPos{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
};

struct PathParameter{
  float a = 0;
  float b = 0;
  float c = 0;
  float d = 0;
};

class IceArmInterface : public rclcpp::Node
{
 public:
  IceArmInterface() : Node("arm_control"), arm_control_("/dev/ttyUSB0", 115200)
  {
  //arm_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/icearm_pos", 10);    // Finger force publisher

    arm_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/icearm_input", 10, std::bind(&IceArmInterface::set_goal_point, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(50ms, std::bind(&IceArmInterface::timer_callback, this));

    GoalPoint.x = 0.00;
    GoalPoint.y = 0.05;
    GoalPoint.z = 0.10;

    RCLCPP_INFO(this->get_logger(), "Setup completed");
  }

 private:
  void timer_callback() {

    // This is now from subscription: set_robot_hand_pos_camera();

    // Update current pos
    CurrentPoint = NextPoint;

    // Set the next point
    if (path_time < goal_time && initialization_complete){
      set_next_point();
    } else {
      NextPoint = GoalPoint;
    }

    // Calculate motor positions
    set_servo_arm_pos();

    // Post new values
    apply_arm_pos();

    if (IceArm_1.arm < -1000){
      std::cout << "------" << std::endl;
    }
  }

  void set_goal_point(geometry_msgs::msg::Twist::SharedPtr msg){
    std::cout << "Point received" << std::endl;

    GoalPoint.x = msg->linear.x;
    GoalPoint.y = msg->linear.y;
    GoalPoint.z = msg->linear.z;

    //goal_time = msg->angular.x;
    initialization_complete = true;

    calculate_path();

    return;
  }

  void set_next_point(){
    NextPoint.x = (ParameterX.a * path_time) + ParameterX.b;
    NextPoint.y = (ParameterY.a * path_time) + ParameterY.b;
    NextPoint.z = (ParameterZ.a * path_time) + ParameterZ.b;

    path_time += 0.05;
  }

  void calculate_path(){

    ParameterX.a = (GoalPoint.x - CurrentPoint.x)/(goal_time);
    ParameterY.a = (GoalPoint.y - CurrentPoint.y)/(goal_time);
    ParameterZ.a = (GoalPoint.z - CurrentPoint.z)/(goal_time);

    ParameterX.b = CurrentPoint.x;
    ParameterY.b = CurrentPoint.y;
    ParameterZ.b = CurrentPoint.z;

    path_time = 0.0;
    return;
  }

  void apply_arm_pos()
  {
    if (IceArm_1.arm < -100 or IceArm_1.forarm < -100){
      RCLCPP_ERROR(this->get_logger(), "Point out of reach");
      exit(0);
    } else {
//      auto arm_msg = geometry_msgs::msg::Twist();
//
//      arm_msg.linear.x = static_cast<int>(IceArm_1.base);
//      arm_msg.linear.y = static_cast<int>(IceArm_1.arm);
//      arm_msg.linear.z = static_cast<int>(IceArm_1.forarm) + 90;
//      arm_msg.angular.x = static_cast<int>(IceArm_1.tool);
//
//      arm_pub_->publish(arm_msg);¨

      std::vector<int> ServoPos = {};
      ServoPos.push_back(static_cast<int>(IceArm_1.base));
      ServoPos.push_back(static_cast<int>(IceArm_1.arm));
      ServoPos.push_back(static_cast<int>(IceArm_1.forarm) + 90);
      ServoPos.push_back(static_cast<int>(IceArm_1.tool));
      ServoPos.push_back(0);
      ServoPos.push_back(0);

      arm_control_.SendServoValues(ServoPos);
    }
  }

  void set_servo_arm_pos()
  {
    // set values for servos
    float total_length = pow(NextPoint.x, 2) + pow(NextPoint.y, 2) + pow(NextPoint.z, 2);

    IceArm_1.base = 57.3f * atan2f(NextPoint.y, NextPoint.x);
    IceArm_1.arm = 114.6f * atan2f(((0.16f * NextPoint.z) + sqrt(-1.0 * total_length * (total_length - 0.0256f))) , ((0.16f * sqrt(pow(NextPoint.x,2) + pow(NextPoint.y,2))) + total_length ));
    IceArm_1.forarm = 114.6f * atan2f(((0.16f * NextPoint.z) - sqrt(-1.0 * total_length * (total_length - 0.0256f))) , ((0.16f * sqrt(pow(NextPoint.x,2) + pow(NextPoint.y,2))) + total_length ));

    //std::cout << IceArm_1.arm << std::endl;
    return;
  }

  // ROS2 and publishers declarations
  rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr arm_sub_;

  // Finger force data struct declaration
  ArmServoPos IceArm_1;

  ToolPos NextPoint;
  ToolPos GoalPoint;
  ToolPos CurrentPoint;

  PathParameter ParameterX;
  PathParameter ParameterY;
  PathParameter ParameterZ;

  float tool_size [2] = {0.55, -0.12};
  float path_time = 0.0;
  float goal_time = 5.0;
  bool initialization_complete = false;

  // Serial communication
  IceSerialServo arm_control_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceArmInterface>());
  rclcpp::shutdown();
  return 0;
}