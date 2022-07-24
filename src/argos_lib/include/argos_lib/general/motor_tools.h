/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include "ctre/Phoenix.h"

/**
 * @brief Calls a motor's `Set()` function while also setting a motor profile to use
 *
 * @param motor The motor object to Set to
 * @param mode Motor control mode. Determines how "value" is interpreted
 * @param value Value to pass to `Set()`
 * @param slotIdx Profile slot to set too
 * @param profileSlot 0 = Primary and 1 = Auxilary Pid Slots for the profile
 */
void SetWithPID(WPI_TalonFX& motor, const ControlMode mode, double value, const int profileSlot, const int pidIdx = 0) {
  motor.SelectProfileSlot(profileSlot, pidIdx);
  motor.Set(mode, value);
}

/**
 * @brief Set all motors given with the same control mode, value, and motor profile info
 *
 * @tparam Motor - Declares a parameter pack
 * @param mode Motor control mode. Determines hos "value" is interpreted
 * @param value Value to pass to `Set()`
 * @param profileSlot Motor profile slot to use
 * @param pidIdx 0 = Primary and 1 = Auxilary Pid Slots for the profile
 * @param motors Variable number of motor objects to set to
 * An example call could be: SetAllMotors(ControlMode::PercentOutput, 0.5, 1, 0, m_motor, m_anotherOne, m_anotherMotor)
 */
template <typename... Motor>
constexpr void SetAllMotors(const ControlMode mode, double value, int profileSlot, int pidIdx, Motor&... motors) {
  // Function shoule never be called with empty pack
  static_assert((sizeof...(motors) > 0), "Call to SetMotors PID must have at least one motor passed");

  // See `SetMotorsPID`
  auto set_Motors =
      [](WPI_TalonFX& m, const ControlMode mode, const double value, const int profileSlot, const int pidIdx) {
        SetWithPID(m, mode, value, profileSlot, pidIdx);
      };
  (set_Motors(motors, mode, value, profileSlot, pidIdx), ...);
}

/**
 * @brief Sets all motors given with the same control mode and value
 *
 * @tparam Motor - Declares parameter pack
 * @param mode Motor control mode. Determines how "value" is interpreted
 * @param value Value to pass to `Set()`
 * @param motors Variable number of motor objects to select set to
 * An example call could be: SetAllMotors(ControlMode::PercentOutput, 0.55, m_motor, m_anotherMotor)
 */
template <typename... Motor>
void SetAllMotors(const ControlMode mode, const double value, Motor&... motors) {
  // Function shoule never be called with empty pack
  static_assert((sizeof...(motors) > 0), "Call to SetMotors PID must have at least one motor passed");

  // See `SetMotorsPID`
  auto set_Motors = [](WPI_TalonFX& m, ControlMode mode, double value) { m.Set(mode, value); };
  (set_Motors(motors, mode, value), ...);
}

/**
 * @brief Sets motor profile slot to use
 *
 * @tparam Motor - Declares parameter pack
 * @param profileSlot Profile to select
 * @param PIDIdx 0 = Primary and 1 = Auxilary Pid Slots for the profile
 * @param motors Variable number of motor objects to select slots of
 * An example call could be: SetMotorsPID(1, 0, m_motorHere, m_anotherMotor, m_anotherMotorPt2)
 */
template <typename... Motor>
void SetMotorsPID(const char profileSlot, const char PIDIdx, Motor&... motors) {
  // Function should never be called with empty pack
  static_assert((sizeof...(motors) > 0), "Call to SetMotors PID must have at least one motor passed");
  // c++17 "fold expression", just a way to unpack a param pack. ",...)" unpacks, and the
  // left side of the "," takes a callable, in this case a lambda for readability and putting more operations in there later
  auto set_motorPID = [](WPI_TalonFX& m, char profileSlot, char PIDIdx) { m.SelectProfileSlot(profileSlot, PIDIdx); };
  (set_motorPID(motors, profileSlot, PIDIdx), ...);
}
