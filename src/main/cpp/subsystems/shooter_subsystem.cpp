/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/shooter_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "argos_lib/config/talonsrx_config.h"
#include "utils/sensor_conversions.h"

ShooterSubsystem::ShooterSubsystem()
    : m_shooterWheelLeft(address::shooter::shooterWheelLeft)
    , m_shooterWheelRight(address::shooter::shooterWheelRight)
    , m_angleControl(address::shooter::angleControl)
    , m_rotationControl(address::shooter::rotationControl)
    , m_hoodHomed(false) {
  argos_lib::falcon_config::FalconConfig<motorConfig::shooter::shooterWheelLeft>(m_shooterWheelLeft, 50_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::shooter::shooterWheelRight>(m_shooterWheelRight, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::shooter::angleControl>(m_angleControl, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::shooter::rotationControl>(m_rotationControl, 50_ms);

  m_shooterWheelRight.Follow(m_shooterWheelLeft);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::AutoAim() {}

void ShooterSubsystem::shooting(double ballfiringspeed) {
  m_shooterWheelLeft.Set(ballfiringspeed);
}

void ShooterSubsystem::ManualAim(double turnSpeed, double hoodSpeed) {
  m_rotationControl.Set(turnSpeed);
  m_angleControl.Set(hoodSpeed);
}

void ShooterSubsystem::UpdateHoodHome() {
  /// @todo use set hood sensor position using SetSelectedSensorPosition method of m_angleControl
  ///       (https://store.ctr-electronics.com/content/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#afc9c5af29bb420fe03d9ef494ee6dda6)
}

bool ShooterSubsystem::IsHoodMoving() {
  /// @todo detect hood speed using GetSelectedSensorVelocity method of m_angleControl
  ///       (https://store.ctr-electronics.com/content/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#a408f0bdeed05461f092a8c64146c075e)
  return false;
}

bool ShooterSubsystem::IsHoodHomed() {
  /// @todo return hood home status
  return false;
}
