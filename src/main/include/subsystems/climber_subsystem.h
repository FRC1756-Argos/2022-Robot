/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <vector>

#include "Constants.h"
#include "argos_lib/config/robot_instance.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "ctre/Phoenix.h"
#include "units/length.h"
#include "utils/sensor_conversions.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  explicit ClimberSubsystem(const argos_lib::RobotInstance instance, const std::vector<ClimberPoint>* preClimbPoints);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
 * @brief manually moves the hook, and/or arm
 *
 * @param hookSpeed moves hook up or down based on the climber a full forward input is 1.0 and a
 *                  backwards input is -1.0
 * @param armSpeed moves arm up or down based on the climber a full up input is 1.0 and a full down input is -1.0
 */
  void ManualControl(double hookSpeed, double armSpeed);

  /**
   * @brief Move hook at specified percent speed
   *
   * @param hookSpeed moves hook up or down based on the climber a full forward input is 1.0 and a
   *                  backwards input is -1.0
   */
  void MoveHook(double hookSpeed);

  /**
   * @brief Move arm at specified percent speed
   *
   * @param armSpeed moves arm up or down based on the climber a full up input is 1.0 and a full down input is -1.0
   */
  void MoveArm(double armSpeed);

  /**
   * @brief Update hook home position
   *
   */
  void UpdateHookHome();

  /**
   * @brief Update arm home position
   *
   */
  void UpdateArmHome();

  /**
   * @brief Sets arm linear actuator positions under closed loop control
   *
   * @param extension Arm position where positive is upward
   * @param profileSlot Profile slot to use on the motor
   */
  void ArmSetPosition(units::inch_t extension, char profileSlot);

  /**
   * @brief Sets arm linear actuator positions under closed loop control using motion magic
   *
   * @param extension Arm position where positive is upward
   * @param cruiseVelocity Max movement velocity
   * @param acceleration Max movement acceleration
   * @param profileSlot Profile slot to use on the motor
   */
  void ArmSetPosition(units::inch_t extension,
                      units::inches_per_second_t cruiseVelocity,
                      units::inches_per_second_squared_t acceleration,
                      char profileSlot);

  /**
   * @brief Set hooks to a given position under closed-loop control
   *
   * @param extension Hook position where 0 is at the shoulder and positive is outward
   * @param profileSlot Profile slot to use on the motor
   */
  void HooksSetPosition(units::inch_t extension, char profileSlot);

  /**
   * @brief Set hooks to a given position under closed-loop control using motion magic
   *
   * @param extension Hook position where 0 is at the shoulder and positive is outward
   * @param cruiseVelocity Max movement velocity
   * @param acceleration Max movement acceleration
   * @param profileSlot Profile slot to use on the motor
   */
  void HooksSetPosition(units::inch_t extension,
                        units::inches_per_second_t cruiseVelocity,
                        units::inches_per_second_squared_t acceleration,
                        char profileSlot);

  /**
   * @brief Detect if hook homing is complete
   *
   * @return True when hook is homed
   */
  bool IsHookHomed();

  /**
   * @brief Detect if arm homing is complete
   *
   * @return True when arm is homed
   */
  bool IsArmHomed();

  /**
   * @brief Detect if hook is in motion
   *
   * @return True when hook is moving
   */
  bool IsHookMoving();

  /**
   * @brief Detect if arm is in motion
   *
   * @return True when arm is in motion
   */
  bool IsArmMoving();

  /**
   * @brief Detect if manual control has been enabled
   *
   * @return True when manual control is active
   */
  bool IsManualOverride();

  /**
   * @brief Runs on robot disable to reset manual control
   */
  void Disable();

  /**
   * @brief Override automatic control
   */
  void ManualOverride();

  /**
   * @brief Enables the hook soft limits
   *
   */
  void SetHookSoftLimits();

  /**
   * @brief Disables hook soft limit
   *
   */
  void DisableHookSoftLimits();

  /**
   * @brief Readies the climber for climb sequence
   *
   */
  void SetClimbReady();

  /**
   * @brief Detect if a value is within a threshold of a target value
   *
   * @tparam T Type that implements operator+(), operator-(), operator<=() and operator>=()
   * @param value Value to check
   * @param target Center of range
   * @param threshold Allowable error from target
   * @return true when value is within threshold of target, false otherwise
   */
  template <typename T>
  constexpr static bool InThreshold(const T value, const T target, const T threshold) {
    return value >= target - threshold && value <= target + threshold;
  }

  void ClimberToSetpoint(ClimberPoint setPoint, char profileSlot);

  void SetClimbMotorsPID(char slot);

  bool ClimberAtPoint(ClimberPoint target);

  // NEW STUFF EVALUATE WHAT NEEDS TO BE REMOVED

  void NextReadyPoint();

  void PreviousReadyPoint();

  void AllowReady();
  void DisallowReady();
  bool IsReadySequenceAllowed();

  bool ClimberReadyToClimb();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_motorLiftRight;
  WPI_TalonFX m_motorLiftLeft;
  WPI_TalonFX m_motorMoveHook;

  bool m_hookHomed;
  bool m_armHomed;

  bool m_allowReady;

  bool m_manualOverride;
  const std::vector<ClimberPoint>* m_pPreClimbPoints;
  std::vector<ClimberPoint>::const_iterator m_itClimberPoint;

  argos_lib::NTMotorPIDTuner m_armPIDTuner;
  argos_lib::NTMotorPIDTuner m_hookPIDTuner;

  bool HooksAtPosition(units::inch_t target);
  bool ArmsAtPosition(units::inch_t target);
};
