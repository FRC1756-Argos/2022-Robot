/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * @brief auto aims shooter to hub
   *
   */
  void AutoAim();

  /**
   * @brief pressing a button to fire a ball
   *
   * @param ballfiringspeed the speed at which the ball shoots out of the shooter 1.0
   */
  void Shoot(double ballfiringspeed);

  /**
   * @brief manually aiming the turret
   *
   * @param turnSpeed turns the turret left or right full right turn is 1.0 full left turn is -1.0
   * @param hoodSpeed move the hood up or down full retract movement is 1.0 full extend movemnt is -1.0
   */
  void ManualAim(double turnSpeed, double hoodSpeed);

  /**
   * @brief Move hood at specified percent speed
   *
   * @param hoodSpeed move the hood up or down full retract movement is 1.0 full extend movemnt is -1.0
   */
  void MoveHood(double hoodSpeed);

  /**
   * @brief Move turret at specified percent speed
   *
   * @param turnSpeed turns the turret left or right full right turn is 1.0 full left turn is -1.0
   */
  void MoveTurret(double turnSpeed);

  /**
   * @brief Update hood home position
   *
   */
  void UpdateHoodHome();

  /**
   * @brief Detect if hood is moving
   *
   * @return True if hood is currently in motion
   */
  bool IsHoodMoving();

  /**
   * @brief Detect if hood homing is complete
   *
   * @return True if hood is homed
   */
  bool IsHoodHomed();

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
  WPI_TalonFX m_shooterWheelLeft;
  WPI_TalonFX m_shooterWheelRight;
  WPI_TalonSRX m_angleControl;
  WPI_TalonSRX m_rotationControl;

  bool m_hoodHomed;  ///< True when hood has known closed loop position
  bool m_manualOverride;
};
