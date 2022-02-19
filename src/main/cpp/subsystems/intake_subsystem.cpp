/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/talonsrx_config.h"
#include "frc/smartdashboard/SmartDashboard.h"

IntakeSubsystem::IntakeSubsystem()
    : m_beltDrive(address::intake::beltDrive)
    , m_intakeDrive(address::intake::intakeDrive)
    , m_intakeDeploy(frc::PneumaticsModuleType::REVPH, address::solenoids::intake)
    , m_ballPresentIntake(address::sensors::tofSensorIntake)
    , m_ballPresentShooter(address::sensors::tofSensorShooter)
    , m_ballColor(address::sensors::colorSensor) {
  // MOTOR CONFIGURATION
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::beltDrive>(m_beltDrive, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::intakeDrive>(m_intakeDrive, 50_ms);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  /// @todo Enable this again once we have sensors
  ///       Otherwise this conflicts with current manual control
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
        m_intakeDrive.Set(speeds::intake::intakeForward);
      } else if (getBallPresent(m_ballPresentIntake) == true && getIsBallTeamColor() == false) {
        m_intakeDrive.Set(speeds::intake::intakeReverse);
      } else {
        m_intakeDrive.Set(0);
      }
      if (m_shooterButtonPressed == true ||
          ((getBallPresent(m_ballPresentIntake) == true && getIsBallTeamColor() == true) &&
           getBallPresent(m_ballPresentShooter) == false)) {
        m_beltDrive.Set(speeds::intake::beltForward);
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
}

units::degree_t IntakeSubsystem::hueModulus(units::degree_t dividend, units::degree_t divisor) {
  return dividend - std::floor(dividend.to<double>() / divisor.to<double>()) * divisor;
}

units::degree_t IntakeSubsystem::hueAngleDifference(units::degree_t referenceAngle, units::degree_t targetAngle) {
  return hueModulus(((referenceAngle - targetAngle) + 180_deg), 360_deg) - 180_deg;
}

bool IntakeSubsystem::getIsBallRed() {
  units::angle::degree_t ballHue = units::make_unit<units::degree_t>(m_ballColor.GetHue());
  double ballSaturation = m_ballColor.GetSaturation();
  // double ballRed = m_ballColor.GetRed();
  return (
      (std::abs(hueAngleDifference(threshholds::colorsensor::hueTargetRed, ballHue).to<double>()) <=
       threshholds::colorsensor::hueRedThresh.to<double>()) &&
      (std::abs(threshholds::colorsensor::satTargetRed - ballSaturation) <= threshholds::colorsensor::satThreshRed)
      // && (std::abs(threshholds::colorsensor::redChannelTarget - ballRed) <= threshholds::colorsensor::redChannelThresh)
  );
}

bool IntakeSubsystem::getIsBallTeamColor() {}

bool IntakeSubsystem::getBallPresent(frc::TimeOfFlight& ballPresentSensor) {
  return false;  //< Replace when sensors are integrated
}

void IntakeSubsystem::StopIntake() {
  m_intakeDeploy.Set(pneumatics::directions::intakeRetract);
  m_intakeDrive.Set(0);
  m_beltDrive.Set(0);
}

void IntakeSubsystem::Intake() {
  m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
  m_intakeDrive.Set(speeds::intake::intakeForward);
  m_beltDrive.Set(speeds::intake::beltForward);
}

void IntakeSubsystem::DumpBall() {
  m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
  m_intakeDrive.Set(speeds::intake::intakeReverse);
  m_beltDrive.Set(speeds::intake::beltReverse);
}

void IntakeSubsystem::ElevatorCycle(bool direction, bool cycleLength) {}
