// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <units/voltage.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration config;
  config.supplyCurrLimit.enable = true;
  config.supplyCurrLimit.currentLimit = 40.0;
  config.statorCurrLimit.enable = true;
  config.statorCurrLimit.currentLimit = 80.0;

  for( TalonFX *motor : {&m_leftShooterMotor, &m_rightShooterMotor} ) {
    motor->ConfigFactoryDefault();
    motor->ConfigAllSettings( config );
    motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  }

  m_leftShooterMotor.SetInverted(true);
  m_rightShooterMotor.Follow(m_leftShooterMotor);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
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
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {talon_ui.Show("TalonFXWidget");}

void Robot::TeleopPeriodic() {talon_ui.Update();}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  m_chooser.SetDefaultOption(kSoftwarePID, kSoftwarePID);
  m_chooser.AddOption(kOnboardPID, kOnboardPID);

  if( m_nt_entries.empty() ) {
      // Only build the entries once (i.e. if m_nt_entries is empty)
    std::string tab = "Shooter Tuning";
      // Motor controls, PID Selection and ENABLE Button
    frc::ShuffleboardLayout &s_layout = frc::Shuffleboard::GetTab(tab)
        .GetLayout("Flywheel Motor Control", frc::BuiltInLayouts::kGrid)
        .WithProperties( wpi::StringMap<std::shared_ptr<nt::Value>>{
                          std::make_pair("Number of rows", nt::Value::MakeDouble(2)),
                          std::make_pair("Number of columns", nt::Value::MakeDouble(2))})
        .WithSize( 6, 4 );
    s_layout.Add("Flywheel Motor", m_leftShooterMotor ).WithPosition(0,0);
    m_nt_entries["Current RPM"] = s_layout.Add("Motor RPM", 0.0 )
        .WithWidget( frc::BuiltInWidgets::kDial )
        .WithProperties( wpi::StringMap<std::shared_ptr<nt::Value>>{
                          std::make_pair("Min", nt::Value::MakeDouble(0)),
                          std::make_pair("Max", nt::Value::MakeDouble(6500))})
        .WithPosition(1,0).GetEntry();
    s_layout.Add("PID Selection", m_chooser).WithPosition(0,1);
    m_nt_entries["Enable"] = s_layout.Add("Enable Motor", false )
        .WithWidget( frc::BuiltInWidgets::kToggleSwitch )
        .WithPosition(1,1).GetEntry();

      // Onboard PID Controls and Error output
    frc::ShuffleboardLayout &ob_layout = frc::Shuffleboard::GetTab(tab)
        .GetLayout("Onboard PID", frc::BuiltInLayouts::kGrid)
        .WithProperties( wpi::StringMap<std::shared_ptr<nt::Value>>{
                          std::make_pair("Number of rows", nt::Value::MakeDouble(2)),
                          std::make_pair("Number of columns", nt::Value::MakeDouble(1))})
        .WithSize( 4, 6 );
    ob_layout.Add("TalonFX Onboard PID", m_onboardPID ).WithPosition(0,0);
    m_nt_entries["OnboardPID Error"] = ob_layout.Add("Onboard PID Error", 0.0 ).WithPosition(0,1).GetEntry();

    frc::ShuffleboardLayout &sw_layout = frc::Shuffleboard::GetTab(tab)
        .GetLayout("Software Feed Forward", frc::BuiltInLayouts::kGrid)
        .WithProperties( wpi::StringMap<std::shared_ptr<nt::Value>>{
                          std::make_pair("Number of rows", nt::Value::MakeDouble(1)),
                          std::make_pair("Number of columns", nt::Value::MakeDouble(2))})
        .WithSize( 6, 5 );
    sw_layout.Add("Software PID", m_softPID ).WithPosition( 0, 0 );

    frc::ShuffleboardLayout &layout = frc::Shuffleboard::GetTab(tab)
        .GetLayout("Software Feed Forward").GetLayout( "FF Settings", frc::BuiltInLayouts::kGrid)
        .WithProperties( wpi::StringMap<std::shared_ptr<nt::Value>>{
                          std::make_pair("Number of rows", nt::Value::MakeDouble(3)),
                          std::make_pair("Number of columns", nt::Value::MakeDouble(2))})
        .WithPosition( 1, 0 )
        .WithSize( 4, 5 );

    m_nt_entries["FF kS"] = layout.Add("Software FF kS", m_ff.kS.value() ).WithPosition(0,0).GetEntry();
    m_nt_entries["FF kV"] = layout.Add("Software FF kV", m_ff.kV.value() ).WithPosition(0,1).GetEntry();
    m_nt_entries["FF kA"] = layout.Add("Software FF kA", m_ff.kA.value() ).WithPosition(0,2).GetEntry();
    m_nt_entries["SoftPID Output"] = layout.Add("PID Output(V)", 0.0 ).WithPosition(1,0).GetEntry();
    m_nt_entries["SoftFF Output"] = layout.Add("FF Output(V)", 0.0 ).WithPosition(1,1).GetEntry();
    m_nt_entries["SoftPID_error"] = layout.Add("Error", 0.0 ).WithPosition(1,2).GetEntry();
}
}

void Robot::TestPeriodic() {
  std::string m_chooserSelected = m_chooser.GetSelected();
  if( m_autoSelected != m_chooserSelected ) {
    // PID controller option has changed.
    m_autoSelected = m_chooserSelected;

    // Stop the motor.
    m_leftShooterMotor.StopMotor();

    if (m_autoSelected == kSoftwarePID) {
        // Using software PID control so turn off Onboard stuff
      m_nt_entries["OnboardPID Error"].SetDouble(0.0);
    } else {
      m_nt_entries["SoftFF Output"].SetDouble( 0.0 );
      m_nt_entries["SoftPID Output"].SetDouble( 0.0 );
    }
    fmt::print("PID selected: {}\n", m_autoSelected);
  }

    // GetSelectedSensorVelocity() returns velocity in tics_per_100ms
    // Need to convert to RPM.
  double curr_rpm = m_leftShooterMotor.GetSelectedSensorVelocity() * kTICS_PER_100MStoRPM;
  m_nt_entries["Current RPM"].SetDouble( curr_rpm );

  if (m_autoSelected == kSoftwarePID) {

      // Calculate() function output is [-1,1] so we need to convert to voltage
    units::volt_t pid_volts = m_softPID.Calculate( curr_rpm, m_softPID.GetSetpoint() ) * 12_V;
    m_nt_entries["SoftPID Output"].SetDouble( pid_volts.value() );

      // NOTE: GetPositionError() returns the difference between the Setpoint and 
      // the current measurement.  In Velocity PID control this is a velocity error
      // so the function is unfortunately named for this case.
    m_nt_entries["SoftPID_error"].SetDouble( m_softPID.GetPositionError() );

      // Get the feed forward constants.
    m_ff.kS = m_nt_entries["FF kS"].GetDouble( 0.0 ) * 1_V;
    m_ff.kV = m_nt_entries["FF kV"].GetDouble( 0.0 ) * 1_V / (1_rad / 1_s);
    m_ff.kA = m_nt_entries["FF kA"].GetDouble( 0.0 ) * 1_V / (1_rad / (1_s * 1_s));

      // Find the Feed forward voltage
    units::volt_t ff_volts = m_ff.Calculate( units::revolutions_per_minute_t(m_softPID.GetSetpoint()) );
    m_nt_entries["SoftFF Output"].SetDouble( ff_volts.value() );

    if( m_nt_entries["Enable"].GetBoolean( false ) ) {
        // Set the motor output voltage to the sum of the two voltages
      m_leftShooterMotor.SetVoltage( pid_volts + ff_volts );
    } else {
      m_leftShooterMotor.SetVoltage( 0_V );
    }
  } else {
    m_nt_entries["OnboardPID Error"].SetDouble( m_leftShooterMotor.GetClosedLoopError() );

    if( m_nt_entries["Enable"].GetBoolean( false ) ) {
        // Setpoint is in RPM, we need tics_per_100ms for the Set() method.
      m_leftShooterMotor.Set( m_onboardPID.GetControl(), m_onboardPID.GetSetpoint() * kRPMtoTICS_PER_100MS );
    } else {
      m_leftShooterMotor.SetVoltage( 0_V );
    }
  }
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
