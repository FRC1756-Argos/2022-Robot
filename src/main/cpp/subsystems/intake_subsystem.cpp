/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>

#include "Constants.h"
#include "argos_lib/config/talonsrx_config.h"
#include "argos_lib/subsystems/swappable_controllers_subsystem.h"

IntakeSubsystem::IntakeSubsystem(const argos_lib::RobotInstance instance,
                                 argos_lib::SwappableControllersSubsystem* controllers)
    : m_beltDriveAddr(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::intake::beltDrive :
                                                                          address::practice_bot::intake::beltDrive)
    , m_intakeDriveAddr(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::intake::intakeDrive :
                                                                            address::practice_bot::intake::intakeDrive)
    , m_beltDrive(m_beltDriveAddr.address)
    , m_intakeDrive(m_intakeDriveAddr.address)
    , m_intakeDeploy(instance == argos_lib::RobotInstance::Competition ? pneumatics::comp_bot::module::moduleAddr :
                                                                         pneumatics::practice::module::moduleAddr,
                     instance == argos_lib::RobotInstance::Competition ? pneumatics::comp_bot::module::moduleType :
                                                                         pneumatics::practice::module::moduleType,
                     instance == argos_lib::RobotInstance::Competition ? address::comp_bot::solenoids::intake :
                                                                         address::practice_bot::solenoids::intake)
    , m_ballPresentIntake(instance == argos_lib::RobotInstance::Competition ?
                              address::comp_bot::sensors::tofSensorIntake :
                              address::practice_bot::sensors::tofSensorIntake)
    , m_ballPresentShooter(instance == argos_lib::RobotInstance::Competition ?
                               address::comp_bot::sensors::tofSensorShooter :
                               address::practice_bot::sensors::tofSensorShooter)
    , m_intakeState(IntakeSubsystem::IntakeState::Stop)
    , m_intakeButtonPressed(false)
    , m_slowIntakeRequested(false)
    , m_outtakeButtonPressed(false)
    , m_shooterButtonPressed(false)
    , m_firstShotMode(false)
    , m_pControllers(controllers)
    // , m_ballColor(address::sensors::colorSensor)
    , m_ShooterEdgeDetector(argos_lib::EdgeDetector::EdgeDetectSettings::DETECT_FALLING)
    , m_IntakeEdgeDetector(argos_lib::EdgeDetector::EdgeDetectSettings::DETECT_RISING)
    , m_hysteresisIntake(threshholds::intake::intakeDeactivate, threshholds::intake::intakeActivate)
    , m_hysteresisShooter(threshholds::intake::intakeDeactivate, threshholds::intake::intakeActivate)
    , m_shooterTimeDebouncer({0_ms, 250_ms}, false) {
  // MOTOR CONFIGURATION
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::beltDrive,
                                             motorConfig::practice_bot::intake::beltDrive>(
      m_beltDrive, 50_ms, instance);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::intakeDrive,
                                             motorConfig::practice_bot::intake::intakeDrive>(
      m_intakeDrive, 50_ms, instance);
}

void IntakeSubsystem::Disable() {
  StopShoot();
  StopIntake();
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  const double intakeingSpeed =
      m_slowIntakeRequested ? speeds::intake::intakeSlowForward : speeds::intake::intakeForward;

  auto currentTime = std::chrono::steady_clock::now();
  static auto lastCalled = currentTime;
  std::chrono::duration<double, std::milli> lastCalledDuration = currentTime - lastCalled;
  double periodicCallSpeed = 1000 / lastCalledDuration.count();
  lastCalled = currentTime;

  bool debouncerStatus = m_shooterTimeDebouncer(m_ShooterEdgeDetector(GetBallPresentShooter()));

  // vibrate controller if ball is at first sensor
  if (m_IntakeEdgeDetector(GetBallPresentIntake())) {
    m_pControllers->DriverController().SetVibration(argos_lib::TemporaryVibrationPattern(
        argos_lib::VibrationConstant(1.0), 500_ms, m_pControllers->DriverController().GetVibration()));
  }

  frc::SmartDashboard::PutNumber("Periodic Speed", periodicCallSpeed);
  frc::SmartDashboard::PutNumber("ToF Distance Intake",
                                 units::inch_t(units::millimeter_t(m_ballPresentIntake.GetRange())).to<double>());
  frc::SmartDashboard::PutNumber("ToF Distance Shooter",
                                 units::inch_t(units::millimeter_t(m_ballPresentShooter.GetRange())).to<double>());

  if (m_outtakeButtonPressed) {
    m_intakeState = IntakeSubsystem::IntakeState::Outtaking;
  } else if (m_intakeButtonPressed || m_shooterButtonPressed) {
    m_intakeState = IntakeSubsystem::IntakeState::Intaking;
  } else {
    m_intakeState = IntakeSubsystem::IntakeState::Stop;
  }
  switch (m_intakeState) {
    case IntakeState::Stop:
      m_intakeDeploy.Set(pneumatics::directions::intakeRetract);
      m_intakeDrive.Set(0);
      m_beltDrive.Set(0);
      break;
    case IntakeState::Intaking:
      if (m_intakeButtonPressed) {
        m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
      } else {
        m_intakeDeploy.Set(pneumatics::directions::intakeRetract);
      }
      if (m_intakeButtonPressed) {
        m_intakeDrive.Set(GetBallPresentIntake() ? speeds::intake::intakeCreep : intakeingSpeed);
      } else if (GetBallPresentIntake() && !GetIsBallTeamColor()) {
        m_intakeDrive.Set(speeds::intake::intakeReverse);
      } else {
        m_intakeDrive.Set(0);
      }
      if ((m_shooterButtonPressed && !debouncerStatus) ||
          ((GetBallPresentIntake() && GetIsBallTeamColor()) && !GetBallPresentShooter())) {
        if (m_shooterButtonPressed) {
          m_beltDrive.Set(m_firstShotMode ? speeds::intake::beltFirstShot : speeds::intake::beltForwardShoot);
        } else {
          m_beltDrive.Set(speeds::intake::beltForwardIntake);
        }
      } else {
        m_beltDrive.Set(0);
      }
      break;
    case IntakeState::Outtaking:
      m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
      m_intakeDrive.Set(speeds::intake::intakeReverse);
      m_beltDrive.Set(speeds::intake::beltReverse);
      break;
  }

  frc::SmartDashboard::PutBoolean("(Ball-Delay) Debouncer Status", debouncerStatus);
}

bool IntakeSubsystem::GetBallPresentIntake() {
  units::inch_t ballDistanceIntake = units::make_unit<units::millimeter_t>(m_ballPresentIntake.GetRange());
  return !m_hysteresisIntake(ballDistanceIntake);
}

bool IntakeSubsystem::GetBallPresentShooter() {
  units::inch_t ballDistanceShooter = units::make_unit<units::millimeter_t>(m_ballPresentShooter.GetRange());
  bool ballPresent = !m_hysteresisShooter(ballDistanceShooter);
  if (ballPresent == true) {
    m_firstShotMode = false;
  }
  return ballPresent;
}

bool IntakeSubsystem::GetIsBallTeamColor() {
  return true;  //< Replace when sensors are integrated
}

void IntakeSubsystem::StopIntake() {
  m_slowIntakeRequested = false;
  m_intakeButtonPressed = false;
  m_outtakeButtonPressed = false;
}

void IntakeSubsystem::SlowIntake() {
  m_slowIntakeRequested = true;
  m_intakeButtonPressed = true;
  m_outtakeButtonPressed = false;
}

void IntakeSubsystem::Intake() {
  m_slowIntakeRequested = false;
  m_intakeButtonPressed = true;
  m_outtakeButtonPressed = false;
}

void IntakeSubsystem::DumpBall() {
  m_slowIntakeRequested = false;
  m_outtakeButtonPressed = true;
  m_intakeButtonPressed = false;
}

void IntakeSubsystem::Shoot() {
  m_shooterButtonPressed = true;
  m_firstShotMode = !GetBallPresentShooter();
}

void IntakeSubsystem::StopShoot() {
  m_shooterButtonPressed = false;
}

void IntakeSubsystem::ElevatorCycle(bool direction, bool cycleLength) {}
