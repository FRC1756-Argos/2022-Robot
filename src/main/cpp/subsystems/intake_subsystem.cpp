/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/talonsrx_config.h"

intake_subsystem::intake_subsystem()
    : beltDrive(address::intake::beltDrive)
    , elevatorIntakeDrive(address::intake::elevatorIntakeDrive)
    , intakeDrive(address::intake::intakeDrive)
    , intakeDeploy(frc::PneumaticsModuleType::REVPH, address::solenoids::intake) {
  // MOTOR CONFIGURATION
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::beltDrive>(beltDrive, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::elevatorIntakeDrive>(elevatorIntakeDrive, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::intake::intakeDrive>(intakeDrive, 50_ms);
}

// This method will be called once per scheduler run
void intake_subsystem::Periodic() {}

void intake_subsystem::Intake() {}

void intake_subsystem::DumpBall() {}

void intake_subsystem::ElevatorCycle(bool direction, bool cycleLength) {}
