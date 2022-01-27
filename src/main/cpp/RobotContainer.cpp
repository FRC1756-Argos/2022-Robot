/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer()
    : m_driveLonSpeedMap(controllerMap::driveLongSpeed)
    , m_driveLatSpeedMap(controllerMap::driveLatSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_hookSpeedMap(controllerMap::hookSpeed)
    , m_armSpeedMap(controllerMap::armSpeed)
    , m_controllers(address::controllers::driver, address::controllers::secondary)
    , m_swerveDrive()
    , m_compressor(frc::PneumaticsModuleType::REVPH) {
  m_compressor.EnableDigital();
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_swerveDrive.SwerveDrive(
            m_driveLonSpeedMap(
                m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand)),
            m_driveLatSpeedMap(
                m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand)),
            m_driveRotSpeed(
                m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kRightHand)));
      },
      {&m_swerveDrive}));

  m_climber.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_climber.manualControl(m_hookSpeedMap(m_controllers.OperatorController().GetX(
                                    argos_lib::XboxController::JoystickHand::kRightHand)),
                                m_armSpeedMap(m_controllers.OperatorController().GetY(
                                    argos_lib::XboxController::JoystickHand::kRightHand)));
      },
      {&m_climber}));
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  auto intake = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
  }});
  auto outtake = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperRight);
  }});
  auto nottake = !intake && !outtake;
  // Configure your button bindings here
  intake.WhenActive([this]() { m_intake.Intake(); }, {&m_intake});
  outtake.WhenActive([this]() { m_intake.DumpBall(); }, {&m_intake});
  nottake.WhenActive([this]() { m_intake.StopIntake(); }, {&m_intake});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
