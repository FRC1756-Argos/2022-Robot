/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <vector>

#include "Constants.h"
#include "argos_lib/config/config_types.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "ctre/Phoenix.h"
#include "units/length.h"
#include "utils/sensor_conversions.h"

/**
 * @brief Controls the climber of the robot
 *
 */
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
   */
  void ArmSetPosition(units::inch_t extension);

  /**
   * @brief Sets arm linear actuator positions under closed loop control using motion magic
   *
   * @param extension Arm position where positive is upward
   * @param cruiseVelocity Max movement velocity
   * @param acceleration Max movement acceleration
   */
  void ArmSetPosition(units::inch_t extension,
                      units::inches_per_second_t cruiseVelocity,
                      units::inches_per_second_squared_t acceleration);

  /**
   * @brief Set hooks to a given position under closed-loop control
   *
   * @param extension Hook position where 0 is at the shoulder and positive is outward
   */
  void HooksSetPosition(units::inch_t extension);

  /**
   * @brief Set hooks to a given position under closed-loop control using motion magic
   *
   * @param extension Hook position where 0 is at the shoulder and positive is outward
   * @param cruiseVelocity Max movement velocity
   * @param acceleration Max movement acceleration
   */
  void HooksSetPosition(units::inch_t extension,
                        units::inches_per_second_t cruiseVelocity,
                        units::inches_per_second_squared_t acceleration);

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

  /**
 * @brief Closed-loop climber to set-point
 *
 * @param setPoint Point to send the climber to
 */
  void ClimberToSetpoint(ClimberPoint setPoint);

  /**
 * @brief Set Climb Motors PID slot to change power
 *
 * @param slot Motor PID slot (see CTRE Falcon 500 docs)
 */
  void SetClimbMotorsPID(char slot);

  /**
 * @brief  Determing if the climber is at a setpoint
 *
 * @param target Target point
 * @return true The climber is at the given point within acceptable error
 * @return false Climber not at point, or outside acceptable error
 */
  bool ClimberAtPoint(ClimberPoint target);

  // NEW STUFF EVALUATE WHAT NEEDS TO BE REMOVED

  /**
   * @brief Go to the next ClimberPoint in the pre-climb sequence
   *
   */
  void NextReadyPoint();

  /**
   * @brief Go to the previous ClimberPoint in the pre-climb sequence
   *
   */
  void PreviousReadyPoint();

  /**
   * @brief Allow climber to go through the ready sequence freely. Publicly exposes m_allowReady
   *
   */
  void AllowReady();

  /**
   * @brief Disable ability to go through ready sequence. Publicly exposes m_allowReady
   *
   */
  void DisallowReady();

  /**
   * @brief Publicly exposes m_allowReady
   *
   * @return true Traversing through the ready sequence is allowed
   * @return false Traversing through the ready sequence is not allowed
   */
  bool IsReadySequenceAllowed();

  /**
   * @brief Determines if climber is able to climb current position
   *
   * @return true Climber is a the end of the ready sequence, can climb
   * @return false Climber is not at the end of the ready sequence, cannot climb
   */
  bool ClimberReadyToClimb();

 private:
  const argos_lib::CANAddress m_motorLiftRightAddr;
  const argos_lib::CANAddress m_motorLiftLeftAddr;
  const argos_lib::CANAddress m_motorMoveHookAddr;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_motorLiftRight;  ///< Right ball-screw driver motor for raising arms
  WPI_TalonFX m_motorLiftLeft;   ///< Left ball-screw driver motor for raising arms
  WPI_TalonFX m_motorMoveHook;   ///< Hook chain drive motor

  bool m_hookHomed;  ///< True if the hooks are homed
  bool m_armHomed;   ///< True if the arms are homed

  bool m_allowReady;  ///< True if the climber can go through ready sequence

  bool m_manualOverride;  ///< True if the operator has manually overriden while closed-loop is running
  const std::vector<ClimberPoint>* m_pPreClimbPoints;  ///< std::vector containing ClimberPoints in the ready sequence
  std::vector<ClimberPoint>::const_iterator
      m_itClimberPoint;  ///< Random-Access iterator that points to a specific ClimberPoint in the readySequence (used as bi-directional)

  argos_lib::NTMotorPIDTuner m_armPIDTuner;   ///< Member for tuning arm PID controls
  argos_lib::NTMotorPIDTuner m_hookPIDTuner;  ///< Member for tuning hook controls

  /**
   * @brief Detects if the hooks are at a particular target
   *
   * @param target Target (in inches) the hooks should be at along their rails
   * @return true Hooks are at target position
   * @return false Hooks are outside target position
   */
  bool HooksAtPosition(units::inch_t target);

  /**
   * @brief  Detects if the arms are at a particular target
   *
   * @param target Target (height in inches) the arms should be at
   * @return true Arms are at target position
   * @return false Arms are outside target position
   */
  bool ArmsAtPosition(units::inch_t target);
};
