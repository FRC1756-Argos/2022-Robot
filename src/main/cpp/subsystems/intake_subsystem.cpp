/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/talonsrx_config.h"

intake_subsystem::intake_subsystem()
    : m_beltDrive(address::intake::beltDrive)
   // , m_elevatorIntakeDrive(address::intake::elevatorIntakeDrive)
    , m_intakeDrive(address::intake::intakeDrive)
    , m_intakeDeploy(frc::PneumaticsModuleType::REVPH, address::solenoids::intake) {
  // MOTOR CONFIGURATION
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::beltDrive>(m_beltDrive, 50_ms);
  //argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::elevatorIntakeDrive>(m_elevatorIntakeDrive, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::intakeDrive>(m_intakeDrive, 50_ms);
}

// This method will be called once per scheduler run
void intake_subsystem::Periodic() {}

void intake_subsystem::StopIntake() {
  m_intakeDeploy.Set(pneumatics::directions::intakeRetract);
  m_intakeDrive.Set(0);
  m_beltDrive.Set(0);
  }

void intake_subsystem::Intake() {
  m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
  m_intakeDrive.Set(speeds::intake::intakeForward);
  m_beltDrive.Set(speeds::intake::beltForward);
  }

void intake_subsystem::DumpBall() {
  m_intakeDeploy.Set(pneumatics::directions::intakeExtend);
  m_intakeDrive.Set(speeds::intake::intakeReverse);
  m_beltDrive.Set(-speeds::intake::beltReverse);
  }

void intake_subsystem::ElevatorCycle(bool direction, bool cycleLength) {}
