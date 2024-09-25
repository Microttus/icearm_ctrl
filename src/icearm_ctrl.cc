//
// Created by biomech on 21.02.24.
//

#include "../include/icearm_ctrl/icearm_ctrl.h"

#include <cmath>
#include <iostream>

IceArmCtrl::IceArmCtrl()
:  path_time(0.0)
, goal_time(5.0)
, initialization_complete(false)
{
  CurrentPoint.x = 0.05;
  CurrentPoint.y = 0.05;
  CurrentPoint.z = 0.05;
}

bool IceArmCtrl::robotRun() {
  // Update current pos
  update_current_pos();

  // Set the next point
  set_next_point_main();

  // Calculate motor positions
  set_servo_arm_pos();

  return true;
}

bool IceArmCtrl::robotOk() {
  if (IceArm_1.arm < -100 or IceArm_1.forarm < -100){
    return false;
  } else {
    return true;
  }
}

bool IceArmCtrl::robotGoal() {
  if (CurrentPoint.x == GoalPoint.x and CurrentPoint.y == GoalPoint.y and CurrentPoint.z == GoalPoint.z) {
    return true;
  } else {
    return false;
  }
}

bool IceArmCtrl::update_current_pos() {
  CurrentPoint = NextPoint;

  return true;
}

bool IceArmCtrl::set_next_point_main() {
  // Set the next point
  if (path_time < goal_time && initialization_complete){
    set_next_point();
  } else {
    NextPoint = GoalPoint;
  }
  return true;
}

bool IceArmCtrl::set_goal_point(float posX, float posY, float posZ){
  // Update pos
  GoalPoint.x = posX;
  GoalPoint.y = posY;
  GoalPoint.z = posZ;

  //goal_time = msg->angular.x;
  initialization_complete = true;

  calculate_path();

  return true;
}

bool IceArmCtrl::set_next_point(){
  NextPoint.x = (ParameterX.a * path_time) + ParameterX.b;
  NextPoint.y = (ParameterY.a * path_time) + ParameterY.b;
  NextPoint.z = (ParameterZ.a * path_time) + ParameterZ.b;

  path_time += 0.01;

  return true;
}

bool IceArmCtrl::calculate_path(){

  ParameterX.a = (GoalPoint.x - CurrentPoint.x)/(goal_time);
  ParameterY.a = (GoalPoint.y - CurrentPoint.y)/(goal_time);
  ParameterZ.a = (GoalPoint.z - CurrentPoint.z)/(goal_time);

  ParameterX.b = CurrentPoint.x;
  ParameterY.b = CurrentPoint.y;
  ParameterZ.b = CurrentPoint.z;

  path_time = 0.0;
  return true;
}

bool IceArmCtrl::set_servo_arm_pos()
{
  // set values for servos
  float total_length = pow(NextPoint.x, 2) + pow(NextPoint.y, 2) + pow(NextPoint.z, 2);

  IceArm_1.base = 57.3f * atan2f(NextPoint.y, NextPoint.x);
  IceArm_1.arm = 114.6f * atan2f(((0.16f * NextPoint.z) + sqrt(-1.0 * total_length * (total_length - 0.0256f))) , ((0.16f * sqrt(pow(NextPoint.x,2) + pow(NextPoint.y,2))) + total_length ));
  IceArm_1.forarm = 114.6f * atan2f(((0.16f * NextPoint.z) - sqrt(-1.0 * total_length * (total_length - 0.0256f))) , ((0.16f * sqrt(pow(NextPoint.x,2) + pow(NextPoint.y,2))) + total_length ));

  //std::cout << IceArm_1.arm << std::endl;
  return true;
}

ArmServoPos IceArmCtrl::return_robot_pos()
{
  return IceArm_1;
}

