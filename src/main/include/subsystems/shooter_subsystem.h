/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "argos_lib/config/robot_instance.h"
#include "argos_lib/general/interpolation.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "ctre/Phoenix.h"
#include "photonlib/PhotonCamera.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "utils/homing_storage_interface.h"

class CameraInterface {
 public:
  CameraInterface();
  photonlib::PhotonCamera m_camera;

  /**
   * @brief Get the highest target the camera can see CAN RETURN NONE
   *
   * @return std::optional<photonlib::PhotonTrackedTarget>
   */
  std::optional<photonlib::PhotonTrackedTarget> GetHighestTarget();

  /**
   * @brief Turns the camera's driver mode on and off
   *
   * @param mode True is drive control. False is no drive control
   */
  void SetDriverMode(bool mode);
};

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  explicit ShooterSubsystem(const argos_lib::RobotInstance instance);

  enum class FixedPosState { Front, Left, Right, Back };

  /// @todo document function
  std::optional<units::degree_t> GetTurretTargetAngle(photonlib::PhotonTrackedTarget target);

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
   * @brief Run shooter at desired speed
   *
   * @param ShooterWheelSpeed Setpoint speed in RPM
   */
  void CloseLoopShoot(units::revolutions_per_minute_t ShooterWheelSpeed);

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
   * @brief Closed loop go to position
   *
   * @param angle with 0 being parallel to ground, positive is raise hood
   */
  void HoodSetPosition(units::degree_t angle);

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
   * @brief Update turret home position
   */
  void UpdateTurretHome();

  /**
   * @brief Initialize turret position based on saved home
   */
  void InitializeTurretHome();

  /**
   * @brief Detect if turret homing is complete
   *
   * @return True if turret is homed
   */
  bool IsTurretHomed();

  /**
   * @brief Closed-loop go to turret position
   *
   * @param angle Setpoint where 0 degrees is intake direction and positive is counterclockwise
   */
  void TurretSetPosition(units::degree_t angle);

  /**
   * @brief Get current angle of turret with 0 degrees at intake side and positive counterclockwise
   *
   * @return Angle of turret if homed, std::nullopt otherwise
   */
  std::optional<units::degree_t> TurretGetPosition();

  /**
   * @brief Sets and enables soft angle limits for turret
   */
  void SetTurretSoftLimits();

  /**
   * @brief Disables soft angle limits for turret
   */
  void DisableTurretSoftLimits();

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
   * @brief Gets from the target to the camera
   *
   * @param targetVerticalAngle The angle of the camera to the ground relative of the target
   * @return Gives the distance in inches of the target to camera
   */
  units::inch_t GetTargetDistance(units::degree_t targetVerticalAngle);

  /**
   * @brief Setting the shooter speed and hood angle depeneding on how far away the target is
   *
   * @param distanceToTarget The distance to the target from the robot
   */
  void SetShooterDistance(units::inch_t distanceToTarget);

  /**
   * @brief Handles fixed close shot shooting positions around hub
   *
   */
  void fixedShooterPosition(FixedPosState);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_shooterWheelLeft;
  WPI_TalonFX m_shooterWheelRight;
  WPI_TalonSRX m_hoodMotor;
  WPI_TalonSRX m_turretMotor;

  CameraInterface m_cameraInterface;

  FSHomingStorage<units::degree_t> m_turretHomingStorage;

  bool m_hoodHomed;    ///< True when hood has known closed loop position
  bool m_turretHomed;  ///< True when turret has known closed loop position
  bool m_manualOverride;

  argos_lib::InterpolationMap<decltype(shooterRange::shooterSpeed.front().inVal), shooterRange::shooterSpeed.size()>
      m_shooterSpeedMap;
  argos_lib::InterpolationMap<decltype(shooterRange::hoodAngle.front().inVal), shooterRange::hoodAngle.size()>
      m_hoodAngleMap;

  argos_lib::NTMotorPIDTuner m_hoodPIDTuner;
  argos_lib::NTMotorPIDTuner m_shooterPIDTuner;
  argos_lib::NTMotorPIDTuner m_turretPIDTuner;

  argos_lib::RobotInstance m_instance;
};
