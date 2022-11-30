//
//      Helper Classes to display CTRE Phoenix motors on Shuffleboard.
//
//  Usage::
//      The tuning::TalonFX class is just a typedef for the WPI_TalonFX class.
//      The tuning::TalonFXPIDController classes are derived
//      from the corresponding ctre:: classes so they can be used in place of them.
//
//      Example Code:
//                      Declare varables in your class for each motor:
//          tuning::TalonFX m_talonMotor{4};
//          tuning::TalonFXPIDController m_tpid{m_talonMotor, ctre::phoenix::motorcontrol::ControlMode::Velocity};
//          tuning::TalonFXWidget talon_ui{m_talonMotor, m_tpid, "Talon Motor 4"};
//
//                      In TestInit() Call the Show() function:
//          void TestInit() {
//              talon_ui.Show( "Motor Test" );
//          }
//
//                      In TestPeriodic() call the Update() function:
//          void TestPeriodic() {
//              talon_ui.Update();
//          }
//
//
//      Individual Widget Usage:
//                      Declare varables in your class:
//          tuning::TalonFX m_talonMotor{4};
//          tuning::TalonFXPIDController m_tpid{m_talonMotor, ctre::phoenix::motorcontrol::ControlMode::Velocity};
//
//                  In RobotInit() or TestInit() send the motor to Smartdashboard or Shuffleboard:
//                      NOTE: Smartdashboard takes a pointer to the object, Shuffleboard does NOT.
//
//          void TestInit() {
//              frc::SmartDashboard::PutData("TalonFX", &m_talonMotor);
//              frc::SmartDashboard::PutData("TalonFX PID", &m_tpid);
//                  .... OR ......
//              frc::Shuffleboard::GetTab("Motors Tab").Add("TalonFX", m_talonMotor );
//              frc::Shuffleboard::GetTab("Motors Tab").Add("TalonFX PID", m_tpid );
//          }
//

#pragma once

#include <string>
#include <regex>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

#undef USE_ENABLE_TOGGLE

// Classes 
namespace tuning {

    // Phoenix Library Provides a class that is Shuffleboard enabled.
    // Just create a typedef to put it in this namespace.
typedef ctre::phoenix::motorcontrol::can::WPI_TalonFX TalonFX;

    // Class for a TalonFX onboard PID controller.
class TalonFXPIDController : public wpi::Sendable, public wpi::SendableHelper<TalonFXPIDController> {
public:
    TalonFXPIDController( TalonFX &talon, ctre::phoenix::motorcontrol::ControlMode mode, int slot = 0 ) 
        : m_talon{talon}, m_slot{slot}, m_mode{mode} {
      m_talon.GetSlotConfigs( config, m_slot, 50 /*timeout in ms*/ );
      zero_config.kP = 0.0;
      zero_config.kI = 0.0;
      zero_config.kD = 0.0;
      zero_config.kF = 0.0;
      UnsetConfig();
    }

    void SetP(double kP) { config.kP=kP; SetConfig();}
    void SetI(double kI) { config.kI=kI; SetConfig(); }
    void SetD(double kD) { config.kD=kD; SetConfig(); }
    void SetFF(double kF) { config.kF=kF; SetConfig(); }
    ctre::phoenix::motorcontrol::ControlMode GetControl( ) {return m_mode;}
    void SetControl(ctre::phoenix::motorcontrol::ControlMode m) {m_mode=m;}
    double GetP() const { return config.kP; }
    double GetI() const { return config.kI; }
    double GetD() const { return config.kD; }
    double GetFF() const { return config.kF; }
    bool GetEnabled() { return m_enabled; }
    void SetEnabled( bool e ) { 
        if( e != m_enabled ) {  // Enabled is changing
            if( e ) { SetConfig(); } else { UnsetConfig(); m_talon.StopMotor(); }
        }
        m_enabled = e; 
    }
    double GetSetpoint() { return m_setpoint; }
    void SetSetpoint( double value) {
        if( m_enabled ) {
                m_talon.Set( m_mode, value );
        } else {
            m_talon.StopMotor();
        }
        m_setpoint = value; 
    }

    void InitSendable(wpi::SendableBuilder& builder) {
        builder.SetSmartDashboardType("PIDController");
        builder.SetActuator(true);
        builder.SetSafeState([&] { m_talon.StopMotor(); });
        builder.AddDoubleProperty(
            "p", [this] { return GetP(); }, [this](double value) { SetP(value); });
        builder.AddDoubleProperty(
            "i", [this] { return GetI(); }, [this](double value) { SetI(value); });
        builder.AddDoubleProperty(
            "d", [this] { return GetD(); }, [this](double value) { SetD(value); });
        builder.AddDoubleProperty(
            "f", [this] { return GetFF(); }, [this](double value) { SetFF(value); });
        builder.AddDoubleProperty(
            "setpoint", [this] { return m_setpoint; },
            [this](double value) { SetSetpoint(value); });
#ifdef USE_ENABLED_TOGGLE
        builder.AddBooleanProperty(
            "enabled", [this] { return GetEnabled(); }, [this](bool e) { SetEnabled(e); });
#endif
    }
private:
    TalonFX &m_talon;
    int m_slot;
    double m_setpoint{0.0};
    bool m_enabled{false};
    ctre::phoenix::motorcontrol::ControlMode m_mode;
    ctre::phoenix::motorcontrol::can::SlotConfiguration config;
    ctre::phoenix::motorcontrol::can::SlotConfiguration zero_config;

    void SetConfig() { m_talon.ConfigureSlot( config, m_slot, 50 /*timeout in ms*/ ); }
    void UnsetConfig() { m_talon.ConfigureSlot( zero_config, m_slot, 50 /*timeout in ms*/ ); }
};

class TalonFXWidget {
public:
    TalonFXWidget( TalonFX &m, TalonFXPIDController &c, std::string name ) : m_talon{m}, m_pid{c}, m_name{name} {}
    void Show( std::string tab ) {
        curr_choice = kVelocityPID;
        m_chooser.SetDefaultOption(kVelocityPID, kVelocityPID);
        m_chooser.AddOption(kPositionPID, kPositionPID);
        m_chooser.AddOption(kPercentOutput, kPercentOutput);

        frc::ShuffleboardLayout &layout = frc::Shuffleboard::GetTab(tab)
            .GetLayout(m_name, frc::BuiltInLayouts::kGrid)
            .WithProperties( wpi::StringMap<std::shared_ptr<nt::Value>>{
                             std::make_pair("Number of columns", nt::Value::MakeDouble(2)),
                             std::make_pair("Number of rows", nt::Value::MakeDouble(2))})
            .WithSize(8, 7);
        layout.Add("Motor Percent Output", m_talon)
            .WithPosition(0, 0);

        layout.Add("Onboard PID Settings", m_pid )
            .WithPosition(0,1);
//        m_pid.SetEnabled( false );

        m_enabled = layout.Add("Enable Motor", false)
            .WithWidget( frc::BuiltInWidgets::kToggleSwitch)
            .WithPosition(1,0).GetEntry();

        frc::ShuffleboardLayout &sub_layout = frc::Shuffleboard::GetTab(tab)
            .GetLayout(m_name).GetLayout("Motor Status", frc::BuiltInLayouts::kGrid)
            .WithProperties( wpi::StringMap<std::shared_ptr<nt::Value>>{
                             std::make_pair("Number of columns", nt::Value::MakeDouble(1)),
                             std::make_pair("Number of rows", nt::Value::MakeDouble(3))})
            .WithPosition(1,1);

        sub_layout.Add("Control Mode", m_chooser )
            .WithWidget(frc::BuiltInWidgets::kComboBoxChooser)
            .WithPosition(0,0);

        m_motor_rpm = sub_layout.Add("Motor RPM", 0.0 )
            .WithPosition(0,1).GetEntry();

        m_motor_pos = sub_layout.Add("Motor Position", 0.0 )
            .WithPosition(0,2).GetEntry();

        wpi::span<double> g_data{graph_data};
        m_graph = &frc::Shuffleboard::GetTab(tab).Add( "Graph", g_data)
                    .WithWidget(frc::BuiltInWidgets::kGraph);
//                    .WithPosition(1, 1)
//                    .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
//                        std::make_pair("Visible time", nt::Value::MakeDouble(20.0)),
//                        std::make_pair("Unit", nt::Value::MakeString(units[0])),
//                        std::make_pair("X-axis auto scrolling", nt::Value::MakeBoolean(false))});

            // This is from Team Jagwires7443 Swerve Code to get the path to the
            // "X-axis auto scrolling" NT item.
        m_graph_metadata = m_graph->GetEntry().GetName();
        m_graph_metadata = std::regex_replace(m_graph_metadata, std::regex("^\\/Shuffleboard\\/"), "/Shuffleboard/.metadata/");
        m_graph_metadata += "/Properties/";
    }

    void Update(void) {        
        graph_data[0] = m_pid.GetSetpoint();
        if( m_chooser.GetSelected() != curr_choice ) {
            if( m_chooser.GetSelected() == kVelocityPID ) {
                m_pid.SetControl( ctre::phoenix::motorcontrol::ControlMode::Velocity );
                nt::NetworkTableInstance::GetDefault().GetEntry(m_graph_metadata + "Unit")
                    .SetString( units[0] );
            } else if( m_chooser.GetSelected() == kPositionPID ) {
                m_pid.SetControl( ctre::phoenix::motorcontrol::ControlMode::Position );
                nt::NetworkTableInstance::GetDefault().GetEntry(m_graph_metadata + "Unit")
                    .SetString( units[1] );
            } else {  /* Percent Output */
                // Nothing to do
            }
            curr_choice = m_chooser.GetSelected();

              // If the Control mode has changed then disable the motor.
              // and switch to enable toggle to OFF.
            m_talon.Disable();
            m_enabled.SetBoolean(false);
        }

        m_motor_pos.SetDouble( m_talon.GetSelectedSensorPosition() / kTICS_PER_TURN );
        m_motor_rpm.SetDouble( m_talon.GetSelectedSensorVelocity() * kTICS_PER_100MStoRPM );
        if( curr_choice == kVelocityPID ) {
            graph_data[1] = m_talon.GetSelectedSensorVelocity() * kTICS_PER_100MStoRPM;
        } else if( curr_choice == kPositionPID ) {
            graph_data[1] = m_talon.GetSelectedSensorPosition() / kTICS_PER_TURN;
        } else {
            // Don't graph
        }
        wpi::span<double> g_data{graph_data};
        m_graph->GetEntry().SetDoubleArray(g_data);

        nt::NetworkTableInstance::GetDefault().GetEntry(m_graph_metadata + "X-axis auto scrolling")
            .SetBoolean( m_enabled.GetBoolean(false) );

        if( m_enabled.GetBoolean(false) ) {
            if( curr_choice == kVelocityPID ) {
                // Sets the target set point on the SparkMAX onboard PID controller.
                m_talon.Set(m_pid.GetControl(), m_pid.GetSetpoint() * kRPMtoTICS_PER_100MS);
            } else if( curr_choice == kPositionPID ) {
                m_talon.Set(m_pid.GetControl(), m_pid.GetSetpoint() * kTICS_PER_TURN);
            } else {  /* Percent Output */
                // Don't Set the percent output.  The motor widget will do so.
            }
        } else {
            m_talon.SetVoltage( 0_V );
        }
    }

private:
    static double constexpr kTICS_PER_TURN = 2048.0;
    static double constexpr kRPMtoTICS_PER_100MS = kTICS_PER_TURN / 600.0;
    static double constexpr kTICS_PER_100MStoRPM = 1.0 / kRPMtoTICS_PER_100MS;

    TalonFX &m_talon;
    TalonFXPIDController m_pid;
    frc::SendableChooser<std::string> m_chooser;
    frc::SimpleWidget *m_graph{nullptr};
    double graph_data[2] = {0.0, 0.0};

    const std::string kVelocityPID = "Velocity"; /*default*/
    const std::string kPositionPID = "Position";
    const std::string kPercentOutput = "Percent Output";
    std::vector<std::string> units = {"\"units\" per 100ms" /*default*/, "\"units\""};
    std::string m_name;
    std::string curr_choice;
    std::string m_graph_metadata;
    nt::NetworkTableEntry m_enabled;
    nt::NetworkTableEntry m_motor_rpm;
    nt::NetworkTableEntry m_motor_pos;
};

}

