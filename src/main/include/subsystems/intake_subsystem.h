/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <TMD37003.h>
#include <TimeOfFlight.h>

#include "argos_lib/config/robot_instance.h"
#include "argos_lib/general/debouncer.h"
#include "argos_lib/general/hysteresis_filter.h"
#include "argos_lib/subsystems/swappable_controllers_subsystem.h"
#include "ctre/Phoenix.h"
#include "frc/Solenoid.h"
#include "utils/edge_detector.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem(const argos_lib::RobotInstance instance, argos_lib::SwappableControllersSubsystem* controller);

  enum class IntakeState { Stop, Intaking, Outtaking };

  /**
   * @brief Handle robot disabling
   */
  void Disable();

  /**
   * @brief Determines whether ball is present at intake
   *
   * @return true - Ball is present at checked sensor
   * @return false - Ball is not present at checked sensor
   */
  bool GetBallPresentIntake();

  /**
   * @brief Determines whether ball is present at shooter
   *
   * @return true - Ball is present at checked sensor
   * @return false - Ball is not present at checked sensor
   */
  bool GetBallPresentShooter();

  /**
   *
   * @todo need to make function not placeholder
   * @brief Determines whether ball is team color
   *
   * @return true - Ball is team color
   * @return false - Ball is not team color
   */
  bool GetIsBallTeamColor();

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
   * @brief Retracts intake. Does not run rollers.
   *
   */
  void StopIntake();

  /**
   * @brief Reverses the intake to dump balls
   *
   */
  void DumpBall();

  /**
   * @brief Shoots ball
   *
   */
  void Shoot();

  /**
   * @brief Stop shooter
   *
   */
  void StopShoot();

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
  WPI_TalonSRX
      m_beltDrive;  ///< Motor that drives the belts on the elevator (Positive towards shooter, Negative away from shooter)
  WPI_TalonSRX m_intakeDrive;  ///< Motor for the intake rollers (Positive is in, Negative is out)

  // SOLENOIDS
  frc::Solenoid m_intakeDeploy;  ///< Solenoid for intake actuation (True is extend, False is retract)

  // SENSORS
  frc::TimeOfFlight m_ballPresentIntake;
  frc::TimeOfFlight m_ballPresentShooter;
  // frc::TMD37003 m_ballColor;  ///< at intake

  // CARGO MANAGEMENT

  IntakeState m_intakeState;

  bool m_intakeButtonPressed;
  bool m_outtakeButtonPressed;
  bool m_shooterButtonPressed;
  bool m_firstShotMode;
  argos_lib::SwappableControllersSubsystem* m_pControllers;

  EdgeDetector m_ShooterEdgeDetector;
  EdgeDetector m_IntakeEdgeDetector;

  argos_lib::HysteresisFilter<units::inch_t> m_hysteresisIntake;
  argos_lib::HysteresisFilter<units::inch_t> m_hysteresisShooter;

  argos_lib::Debouncer m_shooterTimeDebouncer;
};
