/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>

#include "Constants.h"
#include "argos_lib/config/talonsrx_config.h"

IntakeSubsystem::IntakeSubsystem(const argos_lib::RobotInstance instance, const argos_lib::XboxController* controller)
    : m_beltDrive(address::intake::beltDrive)
    , m_intakeDrive(address::intake::intakeDrive)
    , m_intakeDeploy(frc::PneumaticsModuleType::REVPH, address::solenoids::intake)
    , m_ballPresentIntake(address::sensors::tofSensorIntake)
    , m_ballPresentShooter(address::sensors::tofSensorShooter)
    // , m_ballColor(address::sensors::colorSensor)
    , m_edgeDetector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING)
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
  auto currentTime = std::chrono::steady_clock::now();
  static auto lastCalled = currentTime;
  std::chrono::duration<double, std::milli> lastCalledDuration = currentTime - lastCalled;
  double periodicCallSpeed = 1000 / lastCalledDuration.count();
  lastCalled = currentTime;

  bool debouncerStatus = m_shooterTimeDebouncer(m_edgeDetector(GetBallPresentShooter()));

  // vibrate controller if ball is at first sensor
  if (GetBallPresentIntake()) {
  } else {
  }

  frc::SmartDashboard::PutNumber("Periodic Speed", periodicCallSpeed);
  frc::SmartDashboard::PutNumber("ToF Distance Intake",
                                 units::inch_t(units::millimeter_t(m_ballPresentIntake.GetRange())).to<double>());
  frc::SmartDashboard::PutNumber("ToF Distance Shooter",
                                 units::inch_t(units::millimeter_t(m_ballPresentShooter.GetRange())).to<double>());

  if (((m_intakeButtonPressed == true || m_shooterButtonPressed == true) && m_outtakeButtonPressed == false)) {
    m_intakeState = IntakeSubsystem::IntakeState::Intaking;
  } else if (m_outtakeButtonPressed == true) {
    m_intakeState = IntakeSubsystem::IntakeState::Outtaking;
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
      if (m_intakeButtonPressed == true) {
        m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
      } else {
        m_intakeDeploy.Set(pneumatics::directions::intakeRetract);
      }
      if (m_intakeButtonPressed == true) {
        m_intakeDrive.Set(GetBallPresentIntake() ? speeds::intake::intakeCreep : speeds::intake::intakeForward);
      } else if (GetBallPresentIntake() == true && GetIsBallTeamColor() == false) {
        m_intakeDrive.Set(speeds::intake::intakeReverse);
      } else {
        m_intakeDrive.Set(0);
      }
      if ((m_shooterButtonPressed == true && !debouncerStatus) ||
          ((GetBallPresentIntake() == true && GetIsBallTeamColor() == true) && GetBallPresentShooter() == false)) {
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
  /*
  m_intakeDeploy.Set(pneumatics::directions::intakeRetract);
  m_intakeDrive.Set(0);
  m_beltDrive.Set(0);
  */
  m_intakeButtonPressed = false;
  m_outtakeButtonPressed = false;
}

void IntakeSubsystem::Intake() {
  /*
  m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
  m_intakeDrive.Set(speeds::intake::intakeForward);
  m_beltDrive.Set(speeds::intake::beltForward);
  */
  m_intakeButtonPressed = true;
  m_outtakeButtonPressed = false;
}

void IntakeSubsystem::DumpBall() {
  /*
  m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
  m_intakeDrive.Set(speeds::intake::intakeReverse);
  m_beltDrive.Set(speeds::intake::beltReverse);
  */
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
