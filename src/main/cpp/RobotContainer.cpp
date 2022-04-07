/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/PortForwarder.h>

#include "argos_lib/commands/swap_controllers_command.h"
#include "constants/climber_setpoints.h"

RobotContainer::RobotContainer()
    : m_pNetworkTable(std::make_shared<NetworkTablesWrapper>())
    , m_driveSpeedMap(controllerMap::driveSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_hookSpeedMap(controllerMap::hookSpeed)
    , m_armSpeedMap(controllerMap::armSpeed)
    , m_turretSpeedMap(controllerMap::turretSpeed)
    , m_hoodSpeedMap(controllerMap::hoodSpeed)
    , m_instance(argos_lib::GetRobotInstance())
    , m_compressor(m_instance == argos_lib::RobotInstance::Competition ? pneumatics::comp_bot::module::moduleAddr :
                                                                         pneumatics::practice::module::moduleAddr,
                   m_instance == argos_lib::RobotInstance::Competition ? pneumatics::comp_bot::module::moduleType :
                                                                         pneumatics::practice::module::moduleType)
    , m_controllers(address::controllers::driver, address::controllers::secondary)
    , m_swerveDrive(m_pNetworkTable, m_instance)
    , m_intake(m_instance)
    , m_pClimber(m_instance == argos_lib::RobotInstance::Competition ?
                     std::make_unique<ClimberSubsystem>(m_instance, &ClimberSetpoints::PreClimb::preClimbSequence) :
                     nullptr)
    , m_shooter(m_instance, &m_swerveDrive, &m_controllers)
    , m_homeHoodCommand(&m_shooter)
    , m_homeClimberArmCommand(m_pClimber.get())
    , m_homeClimberHookCommand(m_pClimber.get())
    , m_climbCommand(m_pClimber.get(), ClimberSequence::sequence)
    , m_hoodTargetPosition(30_deg)
    , m_shooterTargetVelocity(3000_rpm)
    , m_climberArmTargetExtension(25_in)
    , m_climberArmCruiseVelocity{10_ips}
    , m_climberArmAccel{10_ips2}
    , m_climberHookTargetExtension(20_in)
    , m_climberHookCruiseVelocity{10_ips}
    , m_climberHookAccel{10_ips2}
    , m_NTMonitor("argos")
    , m_driveFollowerLinearkP(m_instance == argos_lib::RobotInstance::Competition ?
                                  controlLoop::comp_bot::drive::linear_follower::kP :
                                  controlLoop::practice_bot::drive::linear_follower::kP)
    , m_driveFollowerLinearkI(m_instance == argos_lib::RobotInstance::Competition ?
                                  controlLoop::comp_bot::drive::linear_follower::kI :
                                  controlLoop::practice_bot::drive::linear_follower::kI)
    , m_driveFollowerLinearkD(m_instance == argos_lib::RobotInstance::Competition ?
                                  controlLoop::comp_bot::drive::linear_follower::kD :
                                  controlLoop::practice_bot::drive::linear_follower::kD)
    , m_driveFollowerRotationalkP(m_instance == argos_lib::RobotInstance::Competition ?
                                      controlLoop::comp_bot::drive::rotational_follower::kP :
                                      controlLoop::practice_bot::drive::rotational_follower::kP)
    , m_driveFollowerRotationalkI(m_instance == argos_lib::RobotInstance::Competition ?
                                      controlLoop::comp_bot::drive::rotational_follower::kI :
                                      controlLoop::practice_bot::drive::rotational_follower::kI)
    , m_driveFollowerRotationalkD(m_instance == argos_lib::RobotInstance::Competition ?
                                      controlLoop::comp_bot::drive::rotational_follower::kD :
                                      controlLoop::practice_bot::drive::rotational_follower::kD)
    , m_driveFollowerRotationalVelocity(m_instance == argos_lib::RobotInstance::Competition ?
                                            controlLoop::comp_bot::drive::rotational_follower::angularVelocity :
                                            controlLoop::practice_bot::drive::rotational_follower::angularVelocity)
    , m_driveFollowerRotationalAcceleration(
          m_instance == argos_lib::RobotInstance::Competition ?
              controlLoop::comp_bot::drive::rotational_follower::angularAcceleration :
              controlLoop::practice_bot::drive::rotational_follower::angularAcceleration)
    , m_driveProfileDistX(120_in)
    , m_driveProfileDistY(0_in)
    , m_driveProfileRot(180_deg)
    , m_driveProfileMaxLinearVel(12_fps)
    , m_driveProfileMaxLinearAccel(units::feet_per_second_squared_t{10})
    , m_driveProfileMaxRotationalVel(60_rpm)
    , m_driveProfileMaxRotationalAccel(units::degrees_per_second_squared_t{360})
    , m_autoRight2Ball{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoRight5Ball{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoRight5BallGap{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoRight5BallDef{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoCenterRight2Ball{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoCenter1ball{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoCenterLeft2Ball{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoCenterLeft2BallDefense{&m_intake, &m_shooter, &m_swerveDrive}
    , m_autoNothing{}
    , m_autoRoutineSelector{{&m_autoRight2Ball,
                             // &m_autoRight5Ball,
                             &m_autoRight5BallGap,
                             // &m_autoRight5BallDef,
                             &m_autoCenterRight2Ball,
                             &m_autoCenter1ball,
                             &m_autoCenterLeft2Ball,
                             &m_autoCenterLeft2BallDefense,
                             &m_autoNothing},
                            &m_autoNothing} {
  // Live window is causing various watchdog timeouts
  frc::LiveWindow::DisableAllTelemetry();

  frc::CameraServer::StartAutomaticCapture();

  // ALLOW ACCESS TO CAMERA STREAM OVER USB
  wpi::PortForwarder::GetInstance().Add(5800, "10.17.56.122", 5800);
  wpi::PortForwarder::GetInstance().Add(5800, "10.17.56.122", 5801);
  wpi::PortForwarder::GetInstance().Add(1181, "10.17.56.122", 1181);
  wpi::PortForwarder::GetInstance().Add(1182, "10.17.56.122", 1182);

  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        const auto deadbandTranslationSpeeds = argos_lib::swerve::CircularInterpolate(
            argos_lib::swerve::TranslationSpeeds{
                -m_controllers.DriverController().GetY(
                    argos_lib::XboxController::JoystickHand::kLeftHand),  // Y axis is negative forward
                -m_controllers.DriverController().GetX(
                    argos_lib::XboxController::JoystickHand::
                        kLeftHand)},  // X axis is positive right, but swerve coordinates are positive left
            m_driveSpeedMap);
        m_swerveDrive.SwerveDrive(
            deadbandTranslationSpeeds.forwardSpeedPct,
            deadbandTranslationSpeeds.leftSpeedPct,
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

  if (m_pClimber) {
    m_pClimber->SetDefaultCommand(frc2::RunCommand(
        [this] {
          m_pClimber->ManualControl(m_hookSpeedMap(m_controllers.OperatorController().GetX(
                                        argos_lib::XboxController::JoystickHand::kRightHand)),
                                    m_armSpeedMap(-m_controllers.OperatorController().GetY(
                                        argos_lib::XboxController::JoystickHand::kRightHand)));
        },
        {m_pClimber.get()}));
  }

  // Robot state triggers
  auto robotEnableTrigger = (frc2::Trigger{[this]() { return frc::DriverStation::IsEnabled(); }});
  auto teleopEnableTrigger = (frc2::Trigger{[this]() { return frc::DriverStation::IsTeleopEnabled(); }});

  // Override triggers
  auto shooterOverrideTrigger = (frc2::Trigger{[this]() {
    return std::abs(m_turretSpeedMap(
               m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand))) > 0 ||
           std::abs(m_hoodSpeedMap(
               m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand))) > 0;
  }});

  auto climberOverrideTrigger = (frc2::Trigger{[this]() {
    return std::abs(m_hookSpeedMap(
               m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kRightHand))) > 0 ||
           std::abs(m_armSpeedMap(
               m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kRightHand))) > 0;
  }});

  // Homing triggers
  auto hoodHomingCompleteTrigger = (frc2::Trigger{[this]() { return m_shooter.IsHoodHomed(); }});
  auto climberHookHomingCompleteTrigger =
      (frc2::Trigger{[this]() { return m_pClimber ? m_pClimber->IsHookHomed() : false; }});
  auto climberArmHomingCompleteTrigger =
      (frc2::Trigger{[this]() { return m_pClimber ? m_pClimber->IsArmHomed() : false; }});

  // Homing commands
  (teleopEnableTrigger && !hoodHomingCompleteTrigger).WhenActive(m_homeHoodCommand);
  if (m_pClimber) {
    (robotEnableTrigger && (!climberArmHomingCompleteTrigger || !climberHookHomingCompleteTrigger))
        .WhenActive(frc2::SequentialCommandGroup(
            m_homeClimberArmCommand,
            m_homeClimberHookCommand,
            frc2::InstantCommand{
                // sets the climber to the first ready setpoint
                [this]() { m_pClimber->ClimberToSetpoint(ClimberSetpoints::PreClimb::preClimbSequence[0]); },
                {m_pClimber.get()}}));
  }

  // Re-enable compressor on enable
  robotEnableTrigger.WhenActive([this]() { m_compressor.EnableDigital(); }, {});

  shooterOverrideTrigger.WhenActive([this]() { m_shooter.ManualOverride(); }, {&m_shooter});
  if (m_pClimber) {
    climberOverrideTrigger.WhenActive([this]() { m_pClimber->ManualOverride(); }, {m_pClimber.get()});
  }

  m_NTMonitor.AddMonitor(
      "manualSetpoints/hoodAngle",
      [this](double newVal) { m_hoodTargetPosition = units::make_unit<units::degree_t>(newVal); },
      m_hoodTargetPosition.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/shooterSpeed",
      [this](double newVal) { m_shooterTargetVelocity = units::make_unit<units::revolutions_per_minute_t>(newVal); },
      m_shooterTargetVelocity.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/armPosition",
      [this](double newVal) { m_climberArmTargetExtension = units::make_unit<units::inch_t>(newVal); },
      m_climberArmTargetExtension.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/armVelocity",
      [this](double newVal) { m_climberArmCruiseVelocity = units::make_unit<units::inches_per_second_t>(newVal); },
      m_climberArmCruiseVelocity.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/armAccel",
      [this](double newVal) { m_climberArmAccel = units::make_unit<units::inches_per_second_squared_t>(newVal); },
      m_climberArmAccel.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/hookPosition",
      [this](double newVal) { m_climberHookTargetExtension = units::make_unit<units::inch_t>(newVal); },
      m_climberHookTargetExtension.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/hookVelocity",
      [this](double newVal) { m_climberHookCruiseVelocity = units::make_unit<units::inches_per_second_t>(newVal); },
      m_climberHookCruiseVelocity.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/hookAccel",
      [this](double newVal) { m_climberHookAccel = units::make_unit<units::inches_per_second_squared_t>(newVal); },
      m_climberHookAccel.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/turretPosition",
      [this](double newVal) { m_turretTargetPosition = units::make_unit<units::degree_t>(newVal); },
      m_turretTargetPosition.to<double>());
  m_NTMonitor.AddMonitor(
      "manualSetpoints/targetShotDistance",
      [this](double newVal) { m_targetShotDistance = units::make_unit<units::inch_t>(newVal); },
      m_targetShotDistance.to<double>());

  // Drive profiling testing
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerLinear/kP",
      [this](double newVal) { m_driveFollowerLinearkP = (newVal); },
      m_driveFollowerLinearkP);
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerLinear/kI",
      [this](double newVal) { m_driveFollowerLinearkI = (newVal); },
      m_driveFollowerLinearkI);
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerLinear/kD",
      [this](double newVal) { m_driveFollowerLinearkD = (newVal); },
      m_driveFollowerLinearkD);
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerRotational/kP",
      [this](double newVal) { m_driveFollowerRotationalkP = (newVal); },
      m_driveFollowerRotationalkP);
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerRotational/kI",
      [this](double newVal) { m_driveFollowerRotationalkI = (newVal); },
      m_driveFollowerRotationalkI);
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerRotational/kD",
      [this](double newVal) { m_driveFollowerRotationalkD = (newVal); },
      m_driveFollowerRotationalkD);
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerRotational/Velocity",
      [this](double newVal) {
        m_driveFollowerRotationalVelocity = units::make_unit<units::degrees_per_second_t>(newVal);
      },
      m_driveFollowerRotationalVelocity.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/FollowerRotational/Acceleration",
      [this](double newVal) {
        m_driveFollowerRotationalAcceleration = units::make_unit<units::degrees_per_second_squared_t>(newVal);
      },
      m_driveFollowerRotationalAcceleration.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/Profile/DistX",
      [this](double newVal) { m_driveProfileDistX = units::make_unit<units::inch_t>(newVal); },
      m_driveProfileDistX.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/Profile/DistY",
      [this](double newVal) { m_driveProfileDistY = units::make_unit<units::inch_t>(newVal); },
      m_driveProfileDistY.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/Profile/Rot",
      [this](double newVal) { m_driveProfileRot = units::make_unit<units::degree_t>(newVal); },
      m_driveProfileRot.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/Profile/MaxLinearVel",
      [this](double newVal) { m_driveProfileMaxLinearVel = units::make_unit<units::feet_per_second_t>(newVal); },
      m_driveProfileMaxLinearVel.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/Profile/MaxLinearAccel",
      [this](double newVal) {
        m_driveProfileMaxLinearAccel = units::make_unit<units::feet_per_second_squared_t>(newVal);
      },
      m_driveProfileMaxLinearAccel.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/Profile/MaxRotationalVel",
      [this](double newVal) { m_driveProfileMaxRotationalVel = units::make_unit<units::degrees_per_second_t>(newVal); },
      m_driveProfileMaxRotationalVel.to<double>());
  m_NTMonitor.AddMonitor(
      "driveProfiling/Profile/MaxRotationalAccel",
      [this](double newVal) {
        m_driveProfileMaxRotationalAccel = units::make_unit<units::degrees_per_second_squared_t>(newVal);
      },
      m_driveProfileMaxRotationalAccel.to<double>());

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

  auto teleopEnableTrigger = (frc2::Trigger{[this]() { return frc::DriverStation::IsTeleopEnabled(); }});

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

  // Fixed Shooting Position Trigger
  auto fixedFrontTrigger = (frc2::Trigger{
      [this]() { return m_controllers.OperatorController().GetRawButton({argos_lib::XboxController::Button::kUp}); }});
  auto fixedLeftTrigger = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetRawButton({argos_lib::XboxController::Button::kLeft});
  }});
  auto fixedRightTrigger = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetRawButton({argos_lib::XboxController::Button::kRight});
  }});
  auto fixedBackTrigger = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetRawButton({argos_lib::XboxController::Button::kDown});
  }});

  // Aiming trigger
  auto aimTrigger = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
  }});
  aimTrigger.WhileActiveContinous([this]() { m_shooter.AutoAim(); }, {&m_shooter});
  aimTrigger.WhenInactive([this]() { m_shooter.SetCameraDriverMode(true); }, {&m_shooter});

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

  // ---------------------------------------------------------------- CLIMBER ----------------------------------------------------------------

  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kDown, {0_ms, 0_ms});

  // frc2::Trigger climbSetPoints{(frc2::Trigger{[this]() {
  //   return m_controllers.OperatorController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
  // }})};

  // climbSetPoints.WhenActive(
  //     [this]() {
  //       m_pClimber->HooksSetPosition(m_climberHookTargetExtension, m_climberHookCruiseVelocity, m_climberHookAccel);
  //       m_pClimber->ArmSetPosition(m_climberArmTargetExtension, m_climberArmCruiseVelocity, m_climberArmAccel);
  //     },
  //     {m_pClimber.get()});

  // climbSetPoints.WhenInactive(
  //     [this]() {
  //       m_pClimber->MoveHook(0);
  //       m_pClimber->MoveArm(0);
  //     },
  //     {m_pClimber.get()});

  frc2::Trigger climbPrevious{(frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButtonPressed(argos_lib::XboxController::Button::kLeft) ||
           m_controllers.OperatorController().GetRawButtonPressed(argos_lib::XboxController::Button::kBumperLeft);
  }})};

  frc2::Trigger climbAdvance{(frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButtonPressed(argos_lib::XboxController::Button::kRight) ||
           m_controllers.OperatorController().GetRawButtonPressed(argos_lib::XboxController::Button::kBumperRight);
  }})};

  frc2::Trigger climbConfirm{(frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetDebouncedButtonPressed(argos_lib::XboxController::Button::kDown);
  }})};

  climbPrevious.WhenActive(
      [this]() {
        if (m_pClimber->IsReadySequenceAllowed()) {
          m_pClimber->PreviousReadyPoint();
        }
      },
      {m_pClimber.get(), &m_shooter});

  climbAdvance.WhenActive(
      [this]() {
        if (m_pClimber->IsReadySequenceAllowed()) {
          m_pClimber->NextReadyPoint();
        }
      },
      {m_pClimber.get()});

  climbConfirm.WhenActive(
      [this]() {
        if (!m_pClimber->ClimberReadyToClimb()) {
          return;
        }
        m_compressor.Disable();
        m_shooter.Disable();
        // CONTROLS WEATHER ROBOT CAN GO THROUGH READY POSITIONS DURING AND AFTER CLIMB
        m_pClimber->DisallowReady();
        m_climbCommand.Schedule();
      },
      {m_pClimber.get(), &m_shooter});

  // TRIGGER ACTIVATION -------------------------------------------------------------------------------------

  // DRIVE TRIGGER ACTIVATION
  controlMode.WhenActive(
      [this]() { m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::robotCentricControl); },
      {&m_swerveDrive});
  controlMode.WhenInactive(
      [this]() { m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl); },
      {&m_swerveDrive});

  fieldHome.WhenActive([this]() { m_swerveDrive.FieldHome(); }, {&m_swerveDrive});

  // INTAKE TRIGGER ACTIVATION
  auto nottake = (!intake) && (!outtake);
  (intake && teleopEnableTrigger).WhileActiveContinous([this]() { m_intake.Intake(); }, {});
  (outtake && teleopEnableTrigger).WhileActiveContinous([this]() { m_intake.DumpBall(); }, {});
  (nottake && teleopEnableTrigger).WhileActiveContinous([this]() { m_intake.StopIntake(); }, {});

  // SHOOTER TRIGGER ACTIVATION
  // aimTrigger.WhenActive(
  //     [this]() {
  //       m_shooter.CloseLoopShoot(m_shooterTargetVelocity);
  //       m_shooter.HoodSetPosition(m_hoodTargetPosition);
  //     },
  //     {&m_shooter});
  (shooter && teleopEnableTrigger).WhileActiveContinous([this]() { m_intake.Shoot(); }, {});
  // aimTrigger.WhenInactive([this]() { m_shooter.Shoot(0); }, {&m_shooter});
  (!shooter && teleopEnableTrigger).WhileActiveContinous([this]() { m_intake.StopShoot(); }, {});

  // SHOOTER FIXED POS TRIGGER ACTIVATION
  fixedFrontTrigger.WhenActive([this]() { m_shooter.FixedShooterPosition(ShooterSubsystem::FixedPosState::Front); },
                               {&m_shooter});
  fixedLeftTrigger.WhenActive([this]() { m_shooter.FixedShooterPosition(ShooterSubsystem::FixedPosState::Left); },
                              {&m_shooter});
  fixedRightTrigger.WhenActive([this]() { m_shooter.FixedShooterPosition(ShooterSubsystem::FixedPosState::Right); },
                               {&m_shooter});
  fixedBackTrigger.WhenActive([this]() { m_shooter.FixedShooterPosition(ShooterSubsystem::FixedPosState::Back); },
                              {&m_shooter});
  // fixedFrontTrigger.WhenActive(
  //     [this]() {
  //       m_shooter.TurretSetPosition(360_deg);
  //       // m_shooter.SetShooterDistance(m_targetShotDistance);
  //       m_shooter.CloseLoopShoot(m_shooterTargetVelocity);
  //       m_shooter.HoodSetPosition(m_hoodTargetPosition);
  //     },
  //     {&m_shooter});
  // fixedLeftTrigger.WhenActive(
  //     [this]() {
  //       m_shooter.TurretSetPosition(90_deg);
  //       // m_shooter.SetShooterDistance(m_targetShotDistance);
  //       m_shooter.CloseLoopShoot(m_shooterTargetVelocity);
  //       m_shooter.HoodSetPosition(m_hoodTargetPosition);
  //     },
  //     {&m_shooter});
  // fixedRightTrigger.WhenActive(
  //     [this]() {
  //       m_shooter.TurretSetPosition(270_deg);
  //       // m_shooter.SetShooterDistance(m_targetShotDistance);
  //       m_shooter.CloseLoopShoot(m_shooterTargetVelocity);
  //       m_shooter.HoodSetPosition(m_hoodTargetPosition);
  //     },
  //     {&m_shooter});
  // fixedBackTrigger.WhenActive(
  //     [this]() {
  //       m_shooter.TurretSetPosition(180_deg);
  //       // m_shooter.SetShooterDistance(m_targetShotDistance);
  //       m_shooter.CloseLoopShoot(m_shooterTargetVelocity);
  //       m_shooter.HoodSetPosition(m_hoodTargetPosition);
  //     },
  //     {&m_shooter});

  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileActiveOnce(argos_lib::SwapControllersCommand(&m_controllers));

  homeDrive.WhenActive([this]() { m_swerveDrive.Home(0_deg); }, {&m_swerveDrive});

  // Drive profile tuning
  // auto driveProfileTrigger = (frc2::Trigger{
  //     [this]() { return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kDown); }});

  // driveProfileTrigger.WhenActive(GetAutonomousCommand());
  //   [this]() {
  //     // m_swerveDrive.UpdateFollowerLinearPIDParams(
  //     //     m_driveFollowerLinearkP, m_driveFollowerLinearkI, m_driveFollowerLinearkD);
  //     // m_swerveDrive.UpdateFollowerRotationalPIDParams(
  //     //     m_driveFollowerRotationalkP, m_driveFollowerRotationalkI, m_driveFollowerRotationalkD);
  //     // m_swerveDrive.UpdateFollowerRotationalPIDConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{
  //     //     m_driveFollowerRotationalVelocity, m_driveFollowerRotationalAcceleration});
  //     // m_swerveDrive.StartDrivingProfile(
  //     //     SwerveTrapezoidalProfileSegment(m_swerveDrive.GetPoseEstimate(),
  //     //                                     frc::Translation2d{m_driveProfileDistX, m_driveProfileDistY},
  //     //                                     frc::Rotation2d{m_driveProfileRot},
  //     //                                     frc::TrapezoidProfile<units::inches>::Constraints{
  //     //                                         m_driveProfileMaxLinearVel, m_driveProfileMaxLinearAccel}));
  //   },
  //   {&m_swerveDrive});

  // driveProfileTrigger.WhenInactive([this]() { GetAutonomousCommand()->Cancel(); }, {&m_swerveDrive});
}

// ----------------------------------------------------------------------------------------------------------

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoRoutineSelector.GetSelectedCommand();
}

void RobotContainer::Disable() {
  m_swerveDrive.Disable();
  m_shooter.Disable();
  m_intake.Disable();
  if (m_pClimber) {
    m_pClimber->Disable();
  }
}
