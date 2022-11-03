/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "frc/smartdashboard/SmartDashboard.h"

ClimberSubsystem::ClimberSubsystem(const argos_lib::RobotInstance instance,
                                   const std::vector<ClimberPoint>* preClimbPoints)
    : m_motorLiftRightAddr(instance == argos_lib::RobotInstance::Competition ?
                               address::comp_bot::climber::liftRight :
                               address::practice_bot::climber::liftRight)
    , m_motorLiftLeftAddr(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::climber::liftLeft :
                                                                              address::practice_bot::climber::liftLeft)
    , m_motorMoveHookAddr(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::climber::moveHook :
                                                                              address::practice_bot::climber::moveHook)
    , m_motorLiftRight(m_motorLiftRightAddr.address, std::string(m_motorLiftRightAddr.busName))
    , m_motorLiftLeft(m_motorLiftLeftAddr.address, std::string(m_motorLiftLeftAddr.busName))
    , m_motorMoveHook(m_motorMoveHookAddr.address, std::string(m_motorMoveHookAddr.busName))
    , m_hookHomed(false)
    , m_armHomed(false)
    , m_allowReady(true)
    , m_manualOverride(false)
    , m_pPreClimbPoints(preClimbPoints)
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

  m_itClimberPoint = m_pPreClimbPoints->begin();
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ManualControl(double hookSpeed, double armSpeed) {
  // Moves system manually, and sets manual override to true
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
  // Update onboard relative encoder to have a pre-defined value at a known position
  m_motorMoveHook.SetSelectedSensorPosition(
      sensor_conversions::climb_hooks::ToSensorUnit(measure_up::climber_hook::homeExtension));
  m_hookHomed = true;
  SetHookSoftLimits();
}

void ClimberSubsystem::UpdateArmHome() {
  // Update onboard relative encoder to have a pre-defined value at a known position
  m_motorLiftLeft.SetSelectedSensorPosition(
      sensor_conversions::climb_arms::ToSensorUnit(measure_up::climber_arm::homeExtension));
  m_motorLiftRight.SetSelectedSensorPosition(
      sensor_conversions::climb_arms::ToSensorUnit(measure_up::climber_arm::homeExtension));
  m_armHomed = true;
}

void ClimberSubsystem::ArmSetPosition(units::inch_t extension) {
  // Closed loop position control, only active when arms are homed
  // see CTRE Motion Magic closed-loop control docs
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
  // Closed loop position control, only active when arms are homed
  // see CTRE Motion Magic closed-loop control docs
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
  // Closed loop position control, only active when hooks are homed
  // see CTRE Motion Magic closed-loop control docs
  if (m_hookHomed) {
    m_manualOverride = false;
    m_motorMoveHook.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                        sensor_conversions::climb_hooks::ToSensorUnit(extension));
  }
}

void ClimberSubsystem::HooksSetPosition(units::inch_t extension,
                                        units::inches_per_second_t cruiseVelocity,
                                        units::inches_per_second_squared_t acceleration) {
  // Closed loop position control, only active when hooks are homed
  // see CTRE Motion Magic closed-loop control docs
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
  // Stop everything
  m_manualOverride = false;
  m_motorLiftRight.Set(0);
  m_motorLiftLeft.Set(0);
  m_motorMoveHook.Set(0);
}

void ClimberSubsystem::ManualOverride() {
  m_manualOverride = true;
}

void ClimberSubsystem::SetHookSoftLimits() {
  // If hooks are homed, set software limits to prevent hardware damage
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
  // Disable software limits of hooks
  m_motorMoveHook.ConfigForwardSoftLimitEnable(false);
  m_motorMoveHook.ConfigReverseSoftLimitEnable(false);
}

void ClimberSubsystem::ClimberToSetpoint(ClimberPoint setPoint) {
  SetClimbMotorsPID(setPoint.pidSlot);
  ArmSetPosition(setPoint.armExtension, setPoint.armSpeed, 20_ips2);
  HooksSetPosition(setPoint.hookExtension, setPoint.hookSpeed, 20_ips2);
}

void ClimberSubsystem::SetClimbMotorsPID(char slot) {
  // useful for using less power during ready, more during climb
  m_motorMoveHook.SelectProfileSlot(slot, 0);
  m_motorLiftLeft.SelectProfileSlot(slot, 0);
  m_motorLiftRight.SelectProfileSlot(slot, 0);
}

bool ClimberSubsystem::HooksAtPosition(units::inch_t target) {
  /* Note that this function can become inacurate as the arms raise. As the linear actuators
  * extend, they cause a small amount of turning on the shaft driving the hooks,
  * without the motor moving, causing the encoder to miss it. This is largely accounted for in the constant
  * values dictating ClimberPoint setpoints.
  */

  // Convert from encoder sensor units to inches
  units::inch_t curPosition = sensor_conversions::climb_hooks::ToExtension(m_motorMoveHook.GetSelectedSensorPosition());
  return InThreshold<units::inch_t>(curPosition, target, 0.125_in);  // 1/8 inch error
}

bool ClimberSubsystem::ArmsAtPosition(units::inch_t target) {
  // Convert from encoder sensor units to inches
  units::inch_t curPosition = sensor_conversions::climb_arms::ToExtension(m_motorLiftRight.GetSelectedSensorPosition());
  return InThreshold<units::inch_t>(curPosition, target, 0.125_in);  // 1/8 inch error
}

bool ClimberSubsystem::ClimberAtPoint(ClimberPoint target) {
  return (ArmsAtPosition(target.armExtension) && HooksAtPosition(target.hookExtension)) ? true : false;
}

// NEW STUFF, EVALUATE WHAT NEEDS TO BE REMOVED ABOVE HERE

void ClimberSubsystem::NextReadyPoint() {
  ++m_itClimberPoint;
  // if the iterator is < the end of the vector,
  // move to the setpoint the iterator points to
  if (m_itClimberPoint < m_pPreClimbPoints->end()) {  // end is right after the last element
    ClimberToSetpoint(*m_itClimberPoint);
  } else {
    --m_itClimberPoint;
  }
}

void ClimberSubsystem::PreviousReadyPoint() {
  --m_itClimberPoint;
  // if the iterator is not < the begining of the vector,
  // move to the setpoint the iterator points to
  if (m_itClimberPoint >= m_pPreClimbPoints->begin()) {  // begin is first element
    ClimberToSetpoint(*m_itClimberPoint);
  } else {
    ++m_itClimberPoint;
  }
}

bool ClimberSubsystem::ClimberReadyToClimb() {
  return ClimberAtPoint(
      ClimberSetpoints::PreClimb::preClimbSequence[ClimberSetpoints::PreClimb::preClimbSequence.size() - 1]);
}

void ClimberSubsystem::AllowReady() {
  m_allowReady = true;
}

void ClimberSubsystem::DisallowReady() {
  m_allowReady = false;
}

bool ClimberSubsystem::IsReadySequenceAllowed() {
  return m_allowReady;
}
