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
  void shooting(double ballfiringspeed);
  /**
  * @brief manually aiming the turret
  *
  * @param turnSpeed turns the turret left or right full right turn is 1.0 full left turn is -1.0
  * @param hoodSpeed move the hood up or down full retract movement is 1.0 full extend movemnt is -1.0
  */
  void ManualAim(double turnSpeed, double hoodSpeed);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_shooterWheelLeft;
  WPI_TalonFX m_shooterWheelRight;
  WPI_TalonSRX m_angleControl;
  WPI_TalonSRX m_rotationControl;
};
