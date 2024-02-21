//
// Created by biomech on 21.02.24.
//

#include "../include/icearm_ctrl/icearm_ctrl.h"



bool IceArmCtrl::update_current_pos() {
  CurrentPoint = NextPoint;
}

void IceArmCtrl::set_next_point_main() {
  // Set the next point
  if (path_time < goal_time && initialization_complete){
    set_next_point();
  } else {
    NextPoint = GoalPoint;
  }
}

void IceArmCtrl::set_goal_point(geometry_msgs::msg::Twist::SharedPtr msg){
  std::cout << "Point received" << std::endl;

  GoalPoint.x = msg->linear.x;
  GoalPoint.y = msg->linear.y;
  GoalPoint.z = msg->linear.z;

  //goal_time = msg->angular.x;
  initialization_complete = true;

  calculate_path();

  return;
}

void IceArmCtrl::set_next_point(){
  NextPoint.x = (ParameterX.a * path_time) + ParameterX.b;
  NextPoint.y = (ParameterY.a * path_time) + ParameterY.b;
  NextPoint.z = (ParameterZ.a * path_time) + ParameterZ.b;

  path_time += 0.01;
}

void IceArmCtrl::calculate_path(){

  ParameterX.a = (GoalPoint.x - CurrentPoint.x)/(goal_time);
  ParameterY.a = (GoalPoint.y - CurrentPoint.y)/(goal_time);
  ParameterZ.a = (GoalPoint.z - CurrentPoint.z)/(goal_time);

  ParameterX.b = CurrentPoint.x;
  ParameterY.b = CurrentPoint.y;
  ParameterZ.b = CurrentPoint.z;

  path_time = 0.0;
  return;
}

void IceArmCtrl::apply_arm_pos()
{
  if (IceArm_1.arm < -100 or IceArm_1.forarm < -100){
    RCLCPP_ERROR(this->get_logger(), "Pont out of reach");
    exit(0);
  } else {
    auto arm_msg = geometry_msgs::msg::Twist();

    arm_msg.linear.x = static_cast<int>(IceArm_1.base);
    arm_msg.linear.y = static_cast<int>(IceArm_1.arm);
    arm_msg.linear.z = static_cast<int>(IceArm_1.forarm);
    arm_msg.angular.x = static_cast<int>(IceArm_1.tool);

    arm_pub_->publish(arm_msg);
  }
}

void IceArmCtrl::set_servo_arm_pos()
{
  // set values for servos
  float total_length = pow(NextPoint.x, 2) + pow(NextPoint.y, 2) + pow(NextPoint.z, 2);

  IceArm_1.base = 57.3f * atan2f(NextPoint.y, NextPoint.x);
  IceArm_1.arm = 114.6f * atan2f(((0.16f * NextPoint.z) + sqrt(-1.0 * total_length * (total_length - 0.0256f))) , ((0.16f * sqrt(pow(NextPoint.x,2) + pow(NextPoint.y,2))) + total_length ));
  IceArm_1.forarm = 114.6f * atan2f(((0.16f * NextPoint.z) - sqrt(-1.0 * total_length * (total_length - 0.0256f))) , ((0.16f * sqrt(pow(NextPoint.x,2) + pow(NextPoint.y,2))) + total_length ));

  //std::cout << IceArm_1.arm << std::endl;
  return;
}