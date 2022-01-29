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
    , m_turretSpeedMap(controllerMap::turretSpeed)
    , m_hoodSpeedMap(controllerMap::hoodSpeed)
    , m_controllers(address::controllers::driver, address::controllers::secondary)
    , m_swerveDrive()
    , m_shooter() {
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
  m_shooter.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_shooter.ManualAim(m_turretSpeedMap(m_controllers.OperatorController().GetX(
                                argos_lib::XboxController::JoystickHand::kLeftHand)),
                            m_hoodSpeedMap(m_controllers.OperatorController().GetY(
                                argos_lib::XboxController::JoystickHand::kLeftHand)));
      },
      {&m_shooter}));

  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  auto shooter = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
  }});
shooter.WhenActive([this](){
  m_shooter.shooting(1);
}, {&m_shooter});
shooter.WhenInactive([this](){
  m_shooter.shooting(0);
}, {&m_shooter});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
