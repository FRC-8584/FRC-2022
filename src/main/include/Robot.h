// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "AHRS.h"
#include "rev/CANSparkMax.h"

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
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

  // frc::PWMSparkMax moto_0{0};
  // frc::PWMSparkMax moto_1{1};
  // frc::PWMSparkMax moto_2{2};
  // frc::PWMSparkMax moto_3{3};

  frc::PWMTalonSRX moto_4{4};
  frc::PWMTalonSRX moto_5{5};
  frc::PWMTalonSRX moto_6{6};

  rev::CANSparkMax motos[4] = {
    rev::CANSparkMax{0, rev::CANSparkMax::MotorType::kBrushed},
    rev::CANSparkMax{1, rev::CANSparkMax::MotorType::kBrushed},
    rev::CANSparkMax{2, rev::CANSparkMax::MotorType::kBrushed},
    rev::CANSparkMax{3, rev::CANSparkMax::MotorType::kBrushed}
  };

  frc::Joystick m_joystick{0};

  AHRS *nav_x = new AHRS(frc::SPI::Port::kMXP);

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
