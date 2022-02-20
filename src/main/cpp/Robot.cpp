// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Tool.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::SmartDashboard::PutNumber("Moto Max Speed", 1);
  frc::SmartDashboard::PutNumber("Climber Speed", 0.3);
  frc::SmartDashboard::PutNumber("Shoot Moto Speed", 0.6);

  frc::SmartDashboard::PutNumber("Shoot Moto", shooter.Get());
  frc::SmartDashboard::PutNumber("Climber_L", climber_L.Get());
  frc::SmartDashboard::PutNumber("Climber_R", climber_R.Get());
  frc::SmartDashboard::PutNumber("Speed_1", moto_1.Get());
  frc::SmartDashboard::PutNumber("Speed_2", moto_2.Get());
  frc::SmartDashboard::PutNumber("Speed_3", moto_3.Get());
  frc::SmartDashboard::PutNumber("Speed_4", moto_4.Get());

  nav_x->ZeroYaw();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double deg, speed, turn, l_speed = 0, r_speed = 0;

  deg = Joystick_Rad(m_joystick.GetRawAxis(0), m_joystick.GetRawAxis(1));
  speed = Joystcik_Speed(m_joystick.GetRawAxis(0), m_joystick.GetRawAxis(1));
  turn = m_joystick.GetRawAxis(4);

  // deg -= DEG_TO_RAD(nav_x->GetYaw());

  if (m_joystick.GetRawButton(7)){
    nav_x->ZeroYaw();
  }

  if (m_joystick.GetRawButton(6)){
    pusher.Set(-0.2);
  }
  else if (m_joystick.GetRawButton(5)){
    pusher.Set(0.2);
  }
  else {
    pusher.Set(0);
  }

  if (m_joystick.GetRawButton(2)){
    taker.Set(0.8);
  }
  else if (m_joystick.GetRawButton(8)){
    taker.Set(-0.8);
  }
  else {
    taker.Set(0);
  }

  double climber_speed = frc::SmartDashboard::GetNumber("Climber Speed", 0.3);
  switch (m_joystick.GetPOV()){
    case 0:
      l_speed = climber_speed;
      r_speed = climber_speed;
      break;
    case 45:
      l_speed = 0;
      r_speed = climber_speed;
      break;
    case 135:
      l_speed = 0;
      r_speed = -1 * climber_speed;
      break;
    case 180:
      l_speed = -1 * climber_speed;
      r_speed = -1 * climber_speed;
      break;
    case 225:
      l_speed = -1 * climber_speed;
      r_speed = 0;
      break;
    case 315:
      l_speed = climber_speed;
      r_speed = 0;
      break;
    default:
      l_speed = 0;
      r_speed = 0;
      break;
  }
  climber_L.Set(l_speed);
  climber_R.Set(r_speed);

  shooter.Set(m_joystick.GetRawAxis(2) * frc::SmartDashboard::GetNumber("Shoot Moto Speed", 0.6));
  
  MecanumControl(deg, speed, turn, this, frc::SmartDashboard::GetNumber("Moto Max Speed", 1));

  frc::SmartDashboard::PutNumber("Shoot Moto", shooter.Get());
  frc::SmartDashboard::PutNumber("Climber_L", climber_L.Get());
  frc::SmartDashboard::PutNumber("Climber_R", climber_R.Get());
  frc::SmartDashboard::PutNumber("Speed_1", moto_1.Get());
  frc::SmartDashboard::PutNumber("Speed_2", moto_2.Get());
  frc::SmartDashboard::PutNumber("Speed_3", moto_3.Get());
  frc::SmartDashboard::PutNumber("Speed_4", moto_4.Get());

  frc::SmartDashboard::PutNumber("navX X", nav_x->GetRawAccelX());
  frc::SmartDashboard::PutNumber("navX Y", nav_x->GetRawAccelY());
  frc::SmartDashboard::PutNumber("navX Z", nav_x->GetRawAccelZ());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
