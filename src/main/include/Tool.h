#include "Robot.h"
#include <math.h>
#include <algorithm>

// 搖桿物理誤差閥值
const double _Joystick_Threshold = 0.05;
// 馬達反轉
const bool _Moto_Reverse[4] = {false, false, false, true};

// 度度量轉弧度量
double DEG_TO_RAD(double num) {
  return num * M_PI / 180;
}

// 弧度量轉度度量
double RAD_TO_DEG(double num) {
  return num * 180 / M_PI;
}

// 搖桿物理誤差修正
double Joystick_Retouch(double val) {
  if (abs(val) < _Joystick_Threshold) {
    return 0;
  }
  else {
    return val;
  }
}

// 搖桿數值轉角度(弧度量)
double Joystick_Rad(double x_val, double y_val) {
  y_val *= -1;
  x_val = Joystick_Retouch(x_val);
  y_val = Joystick_Retouch(y_val);
  if (x_val == 0) {
    if (y_val < 0) {
      return -1 * M_PI / 2;
    }
    else {
      return M_PI / 2;
    }
  }
  else if (y_val == 0) {
    if (x_val < 0) {
      return M_PI;
    }
    else {
      return 0;
    }
  }
  else {
  return atan(y_val / x_val);
  }
}

// 搖桿數值轉速度(0~1)
double Joystcik_Speed(double x_val, double y_val) {
  double retouch_x, retouch_y, raw_speed, max_speed, offset;
  x_val = abs(Joystick_Retouch(x_val));
  y_val = abs(Joystick_Retouch(y_val));
  raw_speed = pow(pow(x_val, 2) + pow(y_val, 2), 0.5);
  if (x_val == 0) {
    return y_val;
  }
  else if (y_val == 0) {
    return x_val;
  }

  if (x_val > y_val) {
    offset = 1 / x_val;
  }
  else {
    offset = 1 / y_val;
  }
  retouch_x = x_val * offset;
  retouch_y = y_val * offset;
  max_speed = pow(pow(retouch_x, 2) + pow(retouch_y, 2), 0.5);
  return raw_speed / max_speed;
}

// 速度等比例合理化
void Speed_Retouch(double val[4]) {
  double offset = 1;
  for (int i = 0; i < 4; i++) {
    if (val[i] != 0) {
      if (1 / abs(val[i]) < offset) {
        offset = 1 / abs(val[i]);
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    val[i] *= offset;
  }
}

// 馬達控制
void MotoControl(double val[4], Robot *robot) {
  robot->moto_0.Set(val[0] * (_Moto_Reverse[0] * 2 - 1));
  robot->moto_1.Set(val[1] * (_Moto_Reverse[1] * 2 - 1));
  robot->moto_2.Set(val[2] * (_Moto_Reverse[2] * 2 - 1));
  robot->moto_3.Set(val[3] * (_Moto_Reverse[3] * 2 - 1));
}

// 麥輪控制
void MecanumControl(double deg, double speed, double turn, Robot *robot) {
  double out_speed[4] = {};
  out_speed[0] = sin(deg) + cos(deg);
  out_speed[1] = sin(deg) - cos(deg);
  out_speed[2] = sin(deg) - cos(deg);
  out_speed[3] = sin(deg) + cos(deg);
  Speed_Retouch(out_speed);
  for (int i = 0; i < 4; i++) {
    out_speed[i] *= speed;
  }
  out_speed[0] += turn;
  out_speed[1] += turn;
  out_speed[2] -= turn;
  out_speed[3] -= turn;
  Speed_Retouch(out_speed);
  MotoControl(out_speed, robot);
}