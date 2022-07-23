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

/**
 * @brief Controls the Intake of the robot and provides internal ball position state info
 *
 */
class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem(const argos_lib::RobotInstance instance, argos_lib::SwappableControllersSubsystem* controller);

  /**
 * @brief Possible intake states
 *
 */
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
   * @brief Deploys intake and starts rollers inwards at slow speed. Handles elevator logic of cycling balls.
   *        This is mostly useful for autonomous where fast intake sometimes spins ball before conveyor
   *
   */
  void SlowIntake();

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
  frc::TimeOfFlight m_ballPresentIntake;   ///< TOF sensor capable of detecting ball at intake
  frc::TimeOfFlight m_ballPresentShooter;  ///< TOF sensor capable of detecting ball at shooter
  // frc::TMD37003 m_ballColor;  ///< at intake

  // CARGO MANAGEMENT

  IntakeState m_intakeState;  ///< Current intake state

  bool m_intakeButtonPressed;                                ///< True if intake button is pressed
  bool m_slowIntakeRequested;                                ///< True if a slow intake is needed
  bool m_outtakeButtonPressed;                               ///< True if the outake button is pressed
  bool m_shooterButtonPressed;                               ///< True if the shooter button is pressed
  bool m_firstShotMode;                                      ///<  True if there was NOT a ball in the shooter
  argos_lib::SwappableControllersSubsystem* m_pControllers;  ///< The driver and operator controllers

  EdgeDetector m_ShooterEdgeDetector;  ///< an edge detector used for detecting a ball leaving the shooter
  EdgeDetector m_IntakeEdgeDetector;   ///< Edge detector detecting a ball entering the intake

  argos_lib::HysteresisFilter<units::inch_t>
      m_hysteresisIntake;  ///< Filter for converting from ball distance to bool, True when TOF sensor reads a ball in front
  argos_lib::HysteresisFilter<units::inch_t>
      m_hysteresisShooter;  ///< Filter for converting from ball distance to bool, True when TOF sensor reads a ball in front

  argos_lib::Debouncer m_shooterTimeDebouncer;  ///< Simple de-bouncer for delaying shooter shots
};
