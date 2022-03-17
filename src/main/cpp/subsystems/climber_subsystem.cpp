/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "frc/smartdashboard/SmartDashboard.h"

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
                        argos_lib::GetSensorConversionFactor(sensor_conversions::climb_arms::ToVelocity),
                        argos_lib::GetSensorConversionFactor(sensor_conversions::climb_arms::ToExtension)}}
    , m_hookPIDTuner{"argos/hook",
                     {&m_motorMoveHook},
                     0,
                     argos_lib::ClosedLoopSensorConversions{
                         argos_lib::GetSensorConversionFactor(sensor_conversions::climb_hooks::ToExtension),
                         argos_lib::GetSensorConversionFactor(sensor_conversions::climb_hooks::ToVelocity),
                         argos_lib::GetSensorConversionFactor(sensor_conversions::climb_hooks::ToExtension)}}
    , m_climberStatus{ClimberStatus::CLIMBER_STORAGE} {
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
  SetHookSoftLimits();
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

void ClimberSubsystem::ArmSetPosition(units::inch_t extension,
                                      units::inches_per_second_t cruiseVelocity,
                                      units::inches_per_second_squared_t acceleration) {
  if (m_armHomed) {
    m_manualOverride = false;
    m_motorLiftLeft.ConfigMotionCruiseVelocity(sensor_conversions::climb_arms::ToSensorVelocity(cruiseVelocity));
    m_motorLiftRight.ConfigMotionCruiseVelocity(sensor_conversions::climb_arms::ToSensorVelocity(cruiseVelocity));
    m_motorLiftLeft.ConfigMotionAcceleration(sensor_conversions::climb_arms::ToSensorAccel(acceleration));
    m_motorLiftRight.ConfigMotionAcceleration(sensor_conversions::climb_arms::ToSensorAccel(acceleration));
    m_motorLiftLeft.Set(ControlMode::MotionMagic, sensor_conversions::climb_arms::ToSensorUnit(extension));
    m_motorLiftRight.Set(ControlMode::MotionMagic, sensor_conversions::climb_arms::ToSensorUnit(extension));
  }
}

void ClimberSubsystem::HooksSetPosition(units::inch_t extension) {
  if (m_hookHomed) {
    m_manualOverride = false;
    m_motorMoveHook.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                        sensor_conversions::climb_hooks::ToSensorUnit(extension));
  }
}

void ClimberSubsystem::HooksSetPosition(units::inch_t extension,
                                        units::inches_per_second_t cruiseVelocity,
                                        units::inches_per_second_squared_t acceleration) {
  if (m_hookHomed) {
    m_manualOverride = false;
    m_motorMoveHook.ConfigMotionCruiseVelocity(sensor_conversions::climb_hooks::ToSensorVelocity(cruiseVelocity));
    m_motorMoveHook.ConfigMotionAcceleration(sensor_conversions::climb_hooks::ToSensorAccel(acceleration));
    m_motorMoveHook.Set(ControlMode::MotionMagic, sensor_conversions::climb_hooks::ToSensorUnit(extension));
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
  m_motorLiftRight.Set(0);
  m_motorLiftLeft.Set(0);
  m_motorMoveHook.Set(0);
}

void ClimberSubsystem::ManualOverride() {
  m_manualOverride = true;
}

void ClimberSubsystem::SetHookSoftLimits() {
  if (m_hookHomed) {
    m_motorMoveHook.ConfigForwardSoftLimitThreshold(
        sensor_conversions::climb_hooks::ToSensorUnit(measure_up::climber_hook::maxExtension));
    m_motorMoveHook.ConfigReverseSoftLimitThreshold(
        sensor_conversions::climb_hooks::ToSensorUnit(measure_up::climber_hook::minExtension));
    m_motorMoveHook.ConfigForwardSoftLimitEnable(true);
    m_motorMoveHook.ConfigReverseSoftLimitEnable(true);
  }
}

void ClimberSubsystem::DisableHookSoftLimits() {
  m_motorMoveHook.ConfigForwardSoftLimitEnable(false);
  m_motorMoveHook.ConfigReverseSoftLimitEnable(false);
}

void ClimberSubsystem::ClimberToSetpoint(ClimberPoint setPoint) {
  SetClimbMotorsPID(setPoint.pidSlot);
  ArmSetPosition(setPoint.armExtension, setPoint.armSpeed, 20_ips2);
  HooksSetPosition(setPoint.hookExtension, setPoint.hookSpeed, 20_ips2);
}

void ClimberSubsystem::SetClimbMotorsPID(char slot) {
  m_motorMoveHook.SelectProfileSlot(slot, 0);
  m_motorLiftLeft.SelectProfileSlot(slot, 0);
  m_motorLiftRight.SelectProfileSlot(slot, 0);
}

bool ClimberSubsystem::HooksAtPosition(units::inch_t target) {
  units::inch_t curPosition = sensor_conversions::climb_hooks::ToExtension(m_motorMoveHook.GetSelectedSensorPosition());
  return InThreshold<units::inch_t>(curPosition, target, 0.125_in);
}

bool ClimberSubsystem::ArmsAtPosition(units::inch_t target) {
  units::inch_t curPosition = sensor_conversions::climb_arms::ToExtension(m_motorLiftRight.GetSelectedSensorPosition());
  return InThreshold<units::inch_t>(curPosition, target, 0.125_in);
}

bool ClimberSubsystem::ClimberAtPoint(ClimberPoint target) {
  return (ArmsAtPosition(target.armExtension) && HooksAtPosition(target.hookExtension)) ? true : false;
}

void ClimberSubsystem::SetClimberReady() {
  if (IsArmMoving() || IsHookMoving()) {
    return;
  }
  ClimberToSetpoint(ClimberSetpoints::setup);
  m_climberStatus = ClimberStatus::CLIMBER_READY;
}

void ClimberSubsystem::SetClimberStorage() {
  if (m_climberStatus == ClimberStatus::CLIMBER_CLIMB) {
    return;
  }

  if (IsArmMoving() || IsHookMoving()) {
    return;
  }
  ClimberToSetpoint(ClimberSetpoints::storage);
  m_climberStatus = ClimberStatus::CLIMBER_STORAGE;
}

void ClimberSubsystem::SetClimberLatch() {
  ClimberToSetpoint(ClimberSetpoints::latchL2);
  m_climberStatus = ClimberStatus::CLIMBER_CLIMB;
}

ClimberSubsystem::ClimberStatus ClimberSubsystem::GetClimberStatus() {
  return m_climberStatus;
}
