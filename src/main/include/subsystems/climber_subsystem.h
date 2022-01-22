// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"

class climber_subsystem : public frc2::SubsystemBase {
 public:
  climber_subsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

/**
 * @brief moves arms up
 *
 */

void armReady();
/**
 * @brief pull hook down
 *
 */

void hookRetract();

/**
 * @brief extend hook along rail
 *
 */

void hookExtend();

/**
 * @brief lower body for center of gravity & hook handoff
 *
 */

void lowerBody();

/**
 * @brief move arm to against bottom of bar
 *
 */

void armToBar();

/**
 * @brief pull body up to stabilize body
 *
 */

void bodyUp();

/**
 * @brief detract hooks, bring arms down
 *
 */

void startingPosition();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
WPI_TalonFX m_motorliftright;
WPI_TalonFX m_motorliftleft;
WPI_TalonFX m_motormovehook;

};
