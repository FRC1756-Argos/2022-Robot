/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer()
    : m_pNetworkTable(std::make_shared<NetworkTablesWrapper>())
    , m_driveLonSpeedMap(controllerMap::driveLongSpeed)
    , m_driveLatSpeedMap(controllerMap::driveLatSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_hookSpeedMap(controllerMap::hookSpeed)
    , m_armSpeedMap(controllerMap::armSpeed)
    , m_turretSpeedMap(controllerMap::turretSpeed)
    , m_hoodSpeedMap(controllerMap::hoodSpeed)
    , m_controllers(address::controllers::driver, address::controllers::secondary)
    , m_swerveDrive(m_pNetworkTable)
//, m_compressor(frc::PneumaticsModuleType::REVPH)
{
  // m_compressor.EnableDigital();
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_swerveDrive.SwerveDrive(
            m_driveLonSpeedMap(
                -m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand)),
            m_driveLatSpeedMap(
                -m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand)),
            m_driveRotSpeed(
                // INVERTED FOR TESTING MAKE SURE TO CORRECT IN REFERENCE FRAME CONVERSION
                -m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kRightHand)));

        // DEBUG STUFF
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left Y",
            m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand));
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left X",
            m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
      },
      {&m_swerveDrive}));
  m_shooter.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_shooter.ManualAim(m_turretSpeedMap(m_controllers.OperatorController().GetX(
                                argos_lib::XboxController::JoystickHand::kLeftHand)),
                            m_hoodSpeedMap(-m_controllers.OperatorController().GetY(
                                argos_lib::XboxController::JoystickHand::kLeftHand)));
      },
      {&m_shooter}));

  m_climber.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_climber.ManualControl(m_hookSpeedMap(m_controllers.OperatorController().GetX(
                                    argos_lib::XboxController::JoystickHand::kRightHand)),
                                m_armSpeedMap(-m_controllers.OperatorController().GetY(
                                    argos_lib::XboxController::JoystickHand::kRightHand)));
      },
      {&m_climber}));
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kX, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kA, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kB, {1500_ms, 0_ms});
  // Configure your button bindings here
  auto intake = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
  }});
  auto outtake = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperRight);
  }});
  auto homeDrive = (frc2::Trigger{[this]() {
    // NOTE: HAD TO REMOVE "COMPLEX" /**/ STYLE COMMENT (WPIFORMAT ERROR ON COMMIT ATTEMPT) THIS CURRENT LINE REFLECTS TESTING, USING A SINGLE BUTTON
    // BE SURE TO CHANGE BACK WHEN NECESSARY
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kA);
  }});
  auto nottake = !intake && !outtake;
  intake.WhenActive([this]() { m_intake.Intake(); }, {&m_intake});
  outtake.WhenActive([this]() { m_intake.DumpBall(); }, {&m_intake});
  nottake.WhenActive([this]() { m_intake.StopIntake(); }, {&m_intake});

  auto shooter = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
  }});
  shooter.WhenActive([this]() { m_shooter.shooting(0.40); }, {&m_shooter});
  shooter.WhenInactive([this]() { m_shooter.shooting(0); }, {&m_shooter});

  homeDrive.WhenActive(
      [this]() {
        std::printf("%d\n", __LINE__);
        m_swerveDrive.Home(0_deg);
      },
      {&m_swerveDrive});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
