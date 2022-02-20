#include "Robot.h"
#include <math.h>
#include <algorithm>

#include <frc/smartdashboard/SmartDashboard.h>

// 搖桿物理誤差閥值常數
const double _JOYSTICK_THRESHOLD = 0;
// 馬達反轉陣列常數
const bool _Moto_Reverse[4] = {false, false, true, true};
// 馬達最大速度
const double _MAX_SPEED = 1;

/** 度度量轉弧度量
 *  @param num 欲轉換數值
 *  @return 弧度量(-π~π)
 */
double DEG_TO_RAD(double num) {
  return num * M_PI / 180;
}

/** 弧度量轉度度量
 *  @param num 欲轉換數值
 *  @return 度度量(-360~360)
 */
double RAD_TO_DEG(double num) {
  return num * 180 / M_PI;
}

/** 搖桿物理誤差修正
 *  @param val 搖桿數值
 *  @return 修正後數值
 */
double Joystick_Retouch(double val) {
  if (abs(val) < _JOYSTICK_THRESHOLD) {
    return 0;
  }
  else {
    return val;
  }
}

/** 搖桿數值轉角度(弧度量)
 *  @param x_val 搖桿X軸數值
 *  @param y_val 搖桿Y軸數值
 *  @return 轉換後弧度量(-π~π)
 */
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
    double vec = pow(pow(x_val, 2) + pow(y_val, 2), 0.5);
    if (y_val > 0) {
      return acos(x_val/vec);
    }
    else {
      return -1 * acos(x_val/vec);
    }
  }
}

/** 搖桿數值轉速度(0~1)
 *  @param x_val 搖桿X軸數值
 *  @param y_val 搖桿Y軸數值
 *  @return 轉換後數值(0~1)
 */
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

/** 速度等比例合理化
 *  @param val[] 速度陣列
 */
void Speed_Retouch(double val[4], bool final = false, double max_speed = -1) {
  double offset = 1, retouch_speed;
  if (max_speed == -1) {
    retouch_speed = _MAX_SPEED;
  }
  else {
    retouch_speed = max_speed;
  }
  for (int i = 0; i < 4; i++) {
    if (val[i] != 0) {
      if (final) {
        if (retouch_speed / abs(val[i]) < offset) {
          offset = retouch_speed / abs(val[i]);
        }
      }
      else {
        if (1 / abs(val[i]) < offset) {
          offset = 1 / abs(val[i]);
        }
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    val[i] *= offset;
  }
}

/** 馬達控制
 *  @param val[] 速度陣列
 *  @param robot Robot Class
 */
void MotoControl(double val[4], Robot *robot) {
  robot->moto_1.Set(val[0] * (_Moto_Reverse[0] * 2 - 1));
  robot->moto_2.Set(val[1] * (_Moto_Reverse[1] * 2 - 1));
  robot->moto_3.Set(val[2] * (_Moto_Reverse[2] * 2 - 1));
  robot->moto_4.Set(val[3] * (_Moto_Reverse[3] * 2 - 1));
}

/** 麥輪控制
 *  @param deg 角度(弧度量)
 *  @param speed 速度
 *  @param turn 轉向速度
 *  @param robot Robot Class
 */
void MecanumControl(double deg, double speed, double turn, Robot *robot, double max_speed = -1) {
  if (max_speed == -1) {
    max_speed = _MAX_SPEED;
  }
  double out_speed[4] = {};
  out_speed[0] = sin(deg) + cos(deg);
  out_speed[1] = sin(deg) - cos(deg);
  out_speed[2] = sin(deg) - cos(deg);
  out_speed[3] = sin(deg) + cos(deg);
  Speed_Retouch(out_speed);
  for (int i = 0; i < 4; i++) {
    out_speed[i] *= speed;
  }
  out_speed[0] += turn * max_speed;
  out_speed[1] += turn * max_speed;
  out_speed[2] -= turn * max_speed;
  out_speed[3] -= turn * max_speed;
  Speed_Retouch(out_speed, true, max_speed);
  MotoControl(out_speed, robot);
}