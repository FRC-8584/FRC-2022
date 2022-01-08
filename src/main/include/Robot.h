// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/Joystick.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

  frc::PWMSparkMax moto_0{0};
  frc::PWMSparkMax moto_1{1};
  frc::PWMSparkMax moto_2{2};
  frc::PWMSparkMax moto_3{3};

  frc::PWMTalonSRX moto_4{4};
  frc::PWMTalonSRX moto_5{5};
  frc::PWMTalonSRX moto_6{6};

  frc::Joystick m_joystick_0{0};
  frc::Joystick m_joystick_1{0};

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
