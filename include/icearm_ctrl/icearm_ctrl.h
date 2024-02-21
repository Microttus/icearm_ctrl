//
// Created by biomech on 21.02.24.
//

#ifndef ROBOTARMWS_SRC_ICEARM_CTRL_SRC_ICEARM_CTRL_HH_
#define ROBOTARMWS_SRC_ICEARM_CTRL_SRC_ICEARM_CTRL_HH_

#include "../icearm_struct.cc"

class IceArmCtrl {
 public:
  IceArmCtrl();
  ~IceArmCtrl() = default;

  bool robotRun();
  bool robotOk();
  bool robotGoal();
  bool set_goal_point(float posX, float posY, float posZ);

  ArmServoPos return_robot_pos();

 private:
  bool update_current_pos();
  bool set_next_point_main();
  bool set_servo_arm_pos();
  bool calculate_path();
  bool set_next_point();

  // Finger force data struct declaration
  ArmServoPos IceArm_1;

  ToolPos NextPoint;
  ToolPos GoalPoint;
  ToolPos CurrentPoint;

  PathParameter ParameterX;
  PathParameter ParameterY;
  PathParameter ParameterZ;

  float tool_size[2] = {0.55, -0.12};
  float path_time;
  float goal_time;
  bool initialization_complete;

};

#endif //ROBOTARMWS_SRC_ICEARM_CTRL_SRC_ICEARM_CTRL_HH_
