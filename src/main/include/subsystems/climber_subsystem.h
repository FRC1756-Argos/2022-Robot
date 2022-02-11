/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "ctre/Phoenix.h"
#include "units/length.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
 * @brief moves arms up
 *
 */

  void ArmReady();
  /**
 * @brief pull hook down
 *
 */

  void hookRetract();

  /**
 * @brief extend hook along rail
 *
 */

  void HookExtend();

  /**
 * @brief lower body for center of gravity & hook handoff
 *
 */

  void LowerBody();

  /**
 * @brief move arm to against bottom of bar
 *
 */

  void ArmToBar();

  /**
 * @brief pull body up to stabilize body
 *
 */

  void BodyUp();

  /**
 * @brief detract hooks, bring arms down
 *
 */

  void StartingPosition();

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
   * @brief Set hooks to a given position under closed-loop control
   *
   * @param extension Hook position where 0 is at the shoulder and positive is outward
   */
  void HooksSetPosition(units::inch_t extension);

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

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_motorLiftRight;
  WPI_TalonFX m_motorLiftLeft;
  WPI_TalonFX m_motorMoveHook;

  bool m_hookHomed;
  bool m_armHomed;

  bool m_manualOverride;

  argos_lib::NTMotorPIDTuner m_armPIDTuner;
  argos_lib::NTMotorPIDTuner m_hookPIDTuner;
};
