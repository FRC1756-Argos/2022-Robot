/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

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

/**
 * @brief manually moves the hook, and/or arm
 *
 * @param hookSpeed moves hook up or down based on the climber a full forward input is 1.0 and a
 *                  backwards input is -1.0
 * @param armSpeed moves arm up or down based on the climber a full up input is 1.0 and a full down input is -1.0
 */
void manualControl(double hookSpeed,double armSpeed);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_motorliftright;
  WPI_TalonFX m_motorliftleft;
  WPI_TalonFX m_motormovehook;
};
