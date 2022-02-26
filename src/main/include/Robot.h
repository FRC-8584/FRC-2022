// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "AHRS.h"
#include "rev/CANSparkMax.h"

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Joystick.h>
#include <frc/SPI.h>

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

  rev::CANSparkMax moto_1{1, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax moto_2{2, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax moto_3{3, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax moto_4{4, rev::CANSparkMax::MotorType::kBrushed};

  rev::CANSparkMax climber_R{6, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax climber_L{7, rev::CANSparkMax::MotorType::kBrushed};

  rev::CANSparkMax shooter{5, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax intaker{8, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax pusher{9, rev::CANSparkMax::MotorType::kBrushed};

  frc::Joystick m_joystick{0};

  AHRS *nav_x = new AHRS(frc::SPI::Port::kMXP);

  double nav_x_offset = 0;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
