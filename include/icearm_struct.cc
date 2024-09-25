//
// Created by biomech on 21.02.24.
//

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