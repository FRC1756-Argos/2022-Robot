/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"
#include "frc/Solenoid.h"

class intake_subsystem : public frc2::SubsystemBase {
 public:
  intake_subsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * @brief Deploys intake and starts rollers inwards. Handles elevator logic of cycling balls.
   *
   */
  void Intake();

  /**
   * @brief Reverses the intake to dump balls
   *
   */
  void DumpBall();

  /**
   * @brief Cycle the elevator to load or dump balls
   *
   * @param direction which direction to run the elevator
   * @param cycleLength the length of the cycle (short/long)
   */
  void ElevatorCycle(bool direction, bool cycleLength);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // MOTORS
  WPI_TalonSRX beltDrive;            ///< Motor that drives the belts on the elevator
  WPI_TalonSRX elevatorIntakeDrive;  ///< Motor for the roller on the front of the elevator
  WPI_TalonSRX intakeDrive;          ///< Motor for the intake rollers

  // SOLENOIDS
  frc::Solenoid intakeDeploy;  ///< Solenoid for intake actuation
};
