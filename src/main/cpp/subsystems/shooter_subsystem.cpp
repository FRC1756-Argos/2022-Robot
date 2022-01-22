/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/shooter_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "argos_lib/config/talonsrx_config.h"

shooter_subsystem::shooter_subsystem()
    : m_shooterWheelLeft(address::shooter::shooterWheelLeft)
    , m_shooterWheelRight(address::shooter::shooterWheelRight)
    , m_angleControl(address::shooter::angleControl)
    , m_rotationControl(address::shooter::rotationControl) {
  argos_lib::falcon_config::FalconConfig<motorConfig::shooter::shooterWheelLeft>(m_shooterWheelLeft, 50_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::shooter::shooterWheelRight>(m_shooterWheelRight, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::shooter::angleControl>(m_angleControl, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::shooter::rotationControl>(m_rotationControl, 50_ms);
}

// This method will be called once per scheduler run
void shooter_subsystem::Periodic() {}

void shooter_subsystem::AutoAim() {}

void shooter_subsystem::shooting() {}

void shooter_subsystem::ManualAim(double turnSpeed, double hoodSpeed) {}
