/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>

#include "argos_lib/commands/swap_controllers_command.h"

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
    , m_homeHoodCommand(&m_shooter)
    , m_homeClimberArmCommand(&m_climber)
    , m_homeClimberHookCommand(&m_climber)
    , m_compressor(frc::PneumaticsModuleType::REVPH) {
  m_compressor.EnableDigital();
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_swerveDrive.SwerveDrive(
            m_driveLonSpeedMap(-m_controllers.DriverController().GetY(
                argos_lib::XboxController::JoystickHand::kLeftHand)),  // Y axis is negative forward
            m_driveLatSpeedMap(-m_controllers.DriverController().GetX(
                argos_lib::XboxController::JoystickHand::
                    kLeftHand)),  // X axis is positive right, but swerve coordiates are positive left
            m_driveRotSpeed(-m_controllers.DriverController().GetX(
                argos_lib::XboxController::JoystickHand::
                    kRightHand)));  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)

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

  // Robot state triggers
  auto robotEnableTrigger = (frc2::Trigger{[this]() { return frc::DriverStation::IsEnabled(); }});

  // Homing triggers
  auto hoodHomingCompleteTrigger = (frc2::Trigger{[this]() { return m_shooter.IsHoodHomed(); }});
  auto climberHookHomingCompleteTrigger = (frc2::Trigger{[this]() { return m_climber.IsHookHomed(); }});
  auto climberArmHomingCompleteTrigger = (frc2::Trigger{[this]() { return m_climber.IsArmHomed(); }});

  // Homing commands
  (robotEnableTrigger && !hoodHomingCompleteTrigger).WhenActive(m_homeHoodCommand);
  (robotEnableTrigger && !climberArmHomingCompleteTrigger).WhenActive(m_homeClimberArmCommand);
  (robotEnableTrigger && !climberHookHomingCompleteTrigger).WhenActive(m_homeClimberHookCommand);

  // Notify subsystems of disable
  robotEnableTrigger.WhenInactive(
      [this]() {
        m_shooter.Disable();
        m_climber.Disable();
      },
      {&m_shooter, &m_climber});

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
    return m_controllers.DriverController().GetDebouncedButton({argos_lib::XboxController::Button::kX,
                                                                argos_lib::XboxController::Button::kA,
                                                                argos_lib::XboxController::Button::kB});
  }});
  auto nottake = !intake && !outtake;
  intake.WhenActive([this]() { m_intake.Intake(); }, {&m_intake});
  outtake.WhenActive([this]() { m_intake.DumpBall(); }, {&m_intake});
  nottake.WhenActive([this]() { m_intake.StopIntake(); }, {&m_intake});

  auto shooter = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
  }});
  shooter.WhenActive([this]() { m_shooter.Shoot(0.40); }, {&m_shooter});
  shooter.WhenInactive([this]() { m_shooter.Shoot(0); }, {&m_shooter});

  // Swap controllers config
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  frc2::Trigger driverTriggerSwapCombo{[this]() {
    return m_controllers.DriverController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};
  frc2::Trigger operatorTriggerSwapCombo{[this]() {
    return m_controllers.OperatorController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};

  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileActiveOnce(argos_lib::SwapControllersCommand(&m_controllers));

  homeDrive.WhenActive([this]() { m_swerveDrive.Home(0_deg); }, {&m_swerveDrive});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
