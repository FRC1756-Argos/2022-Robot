/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "utils/sensor_conversions.h"

ClimberSubsystem::ClimberSubsystem(const argos_lib::RobotInstance instance)
    : m_motorLiftRight(address::climber::liftRight)
    , m_motorLiftLeft(address::climber::liftLeft)
    , m_motorMoveHook(address::climber::moveHook)
    , m_hookHomed(false)
    , m_armHomed(false)
    , m_manualOverride(false)
    , m_armPIDTuner{"argos/arm",
                    {&m_motorLiftLeft, &m_motorLiftRight},
                    0,
                    argos_lib::ClosedLoopSensorConversions{
                        argos_lib::GetSensorConversionFactor(sensor_conversions::climb_arms::ToExtension),
                        1.0,
                        argos_lib::GetSensorConversionFactor(sensor_conversions::climb_arms::ToExtension)}}
    , m_hookPIDTuner{"argos/hook",
                     {&m_motorMoveHook},
                     0,
                     argos_lib::ClosedLoopSensorConversions{
                         argos_lib::GetSensorConversionFactor(sensor_conversions::climb_hooks::ToExtension),
                         1.0,
                         argos_lib::GetSensorConversionFactor(sensor_conversions::climb_hooks::ToExtension)}} {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::liftRight,
                                         motorConfig::practice_bot::climber::liftRight>(
      m_motorLiftRight, 50_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::liftLeft,
                                         motorConfig::practice_bot::climber::liftLeft>(
      m_motorLiftLeft, 50_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::moveHook,
                                         motorConfig::practice_bot::climber::moveHook>(
      m_motorMoveHook, 50_ms, instance);
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ArmReady() {}

void ClimberSubsystem::HookExtend() {}

void ClimberSubsystem::LowerBody() {}

void ClimberSubsystem::ArmToBar() {}

void ClimberSubsystem::BodyUp() {}

void ClimberSubsystem::StartingPosition() {}

void ClimberSubsystem::ManualControl(double hookSpeed, double armSpeed) {
  if (hookSpeed != 0 || armSpeed != 0) {
    m_manualOverride = true;
  }
  if (m_manualOverride) {
    MoveHook(hookSpeed);
    MoveArm(armSpeed);
  }
}

void ClimberSubsystem::MoveHook(double hookSpeed) {
  m_motorMoveHook.Set(hookSpeed);
}

void ClimberSubsystem::MoveArm(double armSpeed) {
  m_motorLiftRight.Set(armSpeed);
  m_motorLiftLeft.Set(armSpeed);
}

void ClimberSubsystem::UpdateHookHome() {
  m_motorMoveHook.SetSelectedSensorPosition(
      sensor_conversions::climb_hooks::ToSensorUnit(measure_up::climber_hook::homeExtension));
  m_hookHomed = true;
}

void ClimberSubsystem::UpdateArmHome() {
  m_motorLiftLeft.SetSelectedSensorPosition(
      sensor_conversions::climb_arms::ToSensorUnit(measure_up::climber_arm::homeExtension));
  m_motorLiftRight.SetSelectedSensorPosition(
      sensor_conversions::climb_arms::ToSensorUnit(measure_up::climber_arm::homeExtension));
  m_armHomed = true;
}

void ClimberSubsystem::ArmSetPosition(units::inch_t extension) {
  if (m_armHomed) {
    m_manualOverride = false;
    m_motorLiftLeft.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                        sensor_conversions::climb_arms::ToSensorUnit(extension));
    m_motorLiftRight.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                         sensor_conversions::climb_arms::ToSensorUnit(extension));
  }
}

void ClimberSubsystem::HooksSetPosition(units::inch_t extension) {
  if (m_armHomed) {
    m_manualOverride = false;
    m_motorMoveHook.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                        sensor_conversions::climb_arms::ToSensorUnit(extension));
  }
}

bool ClimberSubsystem::IsHookHomed() {
  return m_hookHomed;
}

bool ClimberSubsystem::IsArmHomed() {
  return m_armHomed;
}

bool ClimberSubsystem::IsHookMoving() {
  return std::abs(m_motorMoveHook.GetSelectedSensorVelocity()) > 10;
}

bool ClimberSubsystem::IsArmMoving() {
  return std::abs(m_motorLiftLeft.GetSelectedSensorVelocity()) > 10 &&
         std::abs(m_motorLiftRight.GetSelectedSensorVelocity()) > 10;
}

bool ClimberSubsystem::IsManualOverride() {
  return m_manualOverride;
}

void ClimberSubsystem::Disable() {
  m_manualOverride = false;
}
