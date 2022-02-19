/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <wpi/PortForwarder.h>

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
    , m_hoodTargetPosition(30_deg)
    , m_shooterTargetVelocity(3000_rpm)
    , m_NTMonitor("argos") {
  // Live window is causing various watchdog timeouts
  frc::LiveWindow::DisableAllTelemetry();

  // ALLOW ACCESS TO CAMERA STREAM OVER USB
  wpi::PortForwarder::GetInstance().Add(5800, "10.17.56.122", 5800);
  wpi::PortForwarder::GetInstance().Add(1181, "10.17.56.122", 1181);
  wpi::PortForwarder::GetInstance().Add(1182, "10.17.56.122", 1182);

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

  // Homing commands
  (robotEnableTrigger && !hoodHomingCompleteTrigger).WhenActive(m_homeHoodCommand);
  // Notify subsystems of disable
  robotEnableTrigger.WhenInactive([this]() { m_shooter.Disable(); }, {&m_shooter});

  m_NTMonitor.AddMonitor(
      "manualSetpoints/hoodAngle",
      [this](double newVal) { m_hoodTargetPosition = units::make_unit<units::degree_t>(newVal); },
      m_hoodTargetPosition.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/shooterSpeed",
      [this](double newVal) { m_shooterTargetVelocity = units::make_unit<units::revolutions_per_minute_t>(newVal); },
      m_shooterTargetVelocity.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/turretPosition",
      [this](double newVal) { m_turretTargetPosition = units::make_unit<units::degree_t>(newVal); },
      m_turretTargetPosition.to<double>());

  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // CONFIGURE DEBOUNCING
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kX, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kA, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kB, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperLeft, {50_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kY, {1500_ms, 0_ms});

  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kX, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kA, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kB, {1500_ms, 0_ms});

  // TRIGGERS -----------------------------------------------------------------------------------------------

  // Configure your button bindings here
  // INTAKE TRIGGERS
  auto intake = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
  }});
  auto outtake = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperRight);
  }});

  // DRIVE TRIGGERS
  auto homeDrive = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetDebouncedButton({argos_lib::XboxController::Button::kX,
                                                                argos_lib::XboxController::Button::kA,
                                                                argos_lib::XboxController::Button::kB});
  }});

  auto controlMode = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperLeft);
  }});

  auto fieldHome = (frc2::Trigger{
      [this]() { return m_controllers.DriverController().GetDebouncedButton(argos_lib::XboxController::Button::kY); }});

  // SHOOTER TRIGGER
  auto shooter = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
  }});
  shooter.WhenActive(
      [this]() {
        m_shooter.CloseLoopShoot(m_shooterTargetVelocity);
        m_shooter.HoodSetPosition(m_hoodTargetPosition);
      },
      {&m_shooter});
  shooter.WhenInactive([this]() { m_shooter.Shoot(0); }, {&m_shooter});

  // Aiming trigger
  auto aimTrigger = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
  }});
  aimTrigger.WhenActive([this]() { m_shooter.TurretSetPosition(m_turretTargetPosition); }, {&m_shooter});

  auto homeTurret = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetDebouncedButton({argos_lib::XboxController::Button::kX,
                                                                  argos_lib::XboxController::Button::kA,
                                                                  argos_lib::XboxController::Button::kB});
  }});
  homeTurret.WhenActive([this]() { m_shooter.UpdateTurretHome(); }, {&m_shooter});

  // Swap controllers config
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  // SWAP CONTROLLER TRIGGERS
  frc2::Trigger driverTriggerSwapCombo{[this]() {
    return m_controllers.DriverController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};
  frc2::Trigger operatorTriggerSwapCombo{[this]() {
    return m_controllers.OperatorController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};

  // --------------------------------------------------------------------------------------------------------

  // TRIGGER ACTIVATION -------------------------------------------------------------------------------------

  // DRIVE TRIGGER ACTIVATION
  controlMode.WhileActiveContinous(
      [this]() { m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::robotCentricControl); },
      {&m_swerveDrive});
  controlMode.WhenInactive(
      [this]() { m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl); },
      {&m_swerveDrive});

  fieldHome.WhenActive([this]() { m_swerveDrive.FiledHome(); }, {&m_swerveDrive});

  // INTAKE TRIGGER ACTIVATION
  auto nottake = !intake && !outtake;
  intake.WhenActive([this]() { m_intake.Intake(); }, {&m_intake});
  outtake.WhenActive([this]() { m_intake.DumpBall(); }, {&m_intake});
  nottake.WhenActive([this]() { m_intake.StopIntake(); }, {&m_intake});

  // SHOOTER TRIGGER ACTIVATION
  shooter.WhenActive([this]() { m_shooter.Shoot(0.40); }, {&m_shooter});
  shooter.WhenInactive([this]() { m_shooter.Shoot(0); }, {&m_shooter});

  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileActiveOnce(argos_lib::SwapControllersCommand(&m_controllers));

  homeDrive.WhenActive([this]() { m_swerveDrive.Home(0_deg); }, {&m_swerveDrive});
}

// ----------------------------------------------------------------------------------------------------------

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
