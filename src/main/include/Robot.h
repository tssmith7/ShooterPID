// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <map>

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/TimedRobot.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "CtreShuffleboard.h"

double constexpr kRPMtoTICS_PER_100MS = 2048.0 / 600.0;
double constexpr kTICS_PER_100MStoRPM = 1.0 / kRPMtoTICS_PER_100MS;

class Robot : public frc::TimedRobot {
  using TalonFX = ctre::phoenix::motorcontrol::can::WPI_TalonFX;
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
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  TalonFX m_rightShooterMotor{15};
  shuffle::TalonFX m_leftShooterMotor{19};
  shuffle::TalonFXPIDController m_onboardPID{m_leftShooterMotor, ctre::phoenix::motorcontrol::ControlMode::Velocity};
  shuffle::TalonFXWidget talon_ui{m_leftShooterMotor, m_onboardPID, "Shooter Motor"};

  frc2::PIDController m_softPID{0,0,0};
  frc::SimpleMotorFeedforward<units::radians> m_ff{0_V, 1_V/532_rpm};

  std::map<std::string, nt::NetworkTableEntry> m_nt_entries;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kOnboardPID = "Onboard PID";
  const std::string kSoftwarePID = "Software PID";
  std::string m_autoSelected;
};
