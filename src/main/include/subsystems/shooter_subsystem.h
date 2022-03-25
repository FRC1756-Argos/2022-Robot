/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "argos_lib/config/robot_instance.h"
#include "argos_lib/general/interpolation.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "argos_lib/subsystems/swappable_controllers_subsystem.h"
#include "ctre/Phoenix.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "photonlib/PhotonCamera.h"
#include "swerve_drive_subsystem.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "utils/homing_storage_interface.h"

class LimelightTarget {
 private:
  units::degree_t m_pitch;
  units::degree_t m_yaw;
  double m_bboxHor;
  double m_bboxVer;
  units::degree_t m_skew;
  int m_area;
  bool m_hasTargets;
  units::millisecond_t m_pipelineLatency;
  constexpr static units::millisecond_t m_miscLatency{11_ms};

 public:
  LimelightTarget() = default;

  /**
   * @brief Describes the values of vision target
   *
   */
  struct tValues {
    units::degree_t pitch;
    units::degree_t yaw;
    double bboxHor;
    double bboxVer;
    units::degree_t skew;
    units::millisecond_t totalLatency;
  };

  /**
   * @brief Get the values of the camera's current target
   *
   * @return tValues
   */
  tValues GetTarget();

  /**
   * @brief Does the camera see a target?
   *
   * @return true - The camera does see a target
   * @return false - The camera does not see a target
   */
  bool HasTarget();
};

/**
 * @brief Shooter aiming parameters
 */
struct AimValues {
  units::degree_t turretTarget;
  units::degree_t hoodTarget;
  units::angular_velocity::revolutions_per_minute_t shooterTarget;
};

class CameraInterface {
 public:
  CameraInterface();

  LimelightTarget m_target;  ///< object that holds the current target

  /**
   * @brief Get the highest target the camera can see CAN RETURN NONE
   *
   * @return std::optional<photonlib::PhotonTrackedTarget>
   */
  std::optional<LimelightTarget> GetHighestTarget();

  /**
   * @brief Converts vertical pixels to degrees
   *
   * @return Degrees
   */
  units::angle::degree_t VerticalPixelToAngle(int pixels);

  /**
   * @brief Converts horizontal pixels to degrees
   *
   * @return Degrees
   */
  units::angle::degree_t HorizontalPixelToAngle(int pixels);

  /**
   * @brief Gets the new pitch for the top of the target
   *
   * @return Pitch degrees of the top of the target
   */
  units::angle::degree_t GetNewPitch(
      units::degree_t cx, units::degree_t cy, int bboxHorizontalPixels, int bboxVerticalPixels, units::degree_t skew);

  /**
   * @brief Turns the camera's driver mode on and off
   *
   * @param mode True is drive control. False is no drive control
   */
  void SetDriverMode(bool mode);
};

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * @brief Hood and shooter setpoints for shooting at a known distance
   */
  struct ShooterDistanceSetpoints {
    units::revolutions_per_minute_t shooterSpeed;
    units::degree_t hoodAngle;
  };

  struct HubRelativeVelocities {
    units::feet_per_second_t radialVelocity;      ///< Positive is velocity toward center of hub
    units::feet_per_second_t tangentialVelocity;  ///< Positive is velocity clockwise around hub
    units::degrees_per_second_t chassisYawRate;   ///< Positive is robot rotating counterclockwise
  };

  /**
   * @brief Offsets useful for aiming while driving
   */
  struct AimOffsets {
    units::foot_t distanceOffset;  ///< Positive indicates aim farther to compensate for motion away from hub
    units::degree_t yawOffset;     ///< Positive indicates aim left of hub to compensate for robot lateral motion
  };

  ShooterSubsystem(const argos_lib::RobotInstance instance,
                   SwerveDriveSubsystem* pDriveSubsystem,
                   argos_lib::SwappableControllersSubsystem* controllers = nullptr);

  enum class FixedPosState { Front, Left, Right, Back };

  /// @todo document function
  std::optional<units::degree_t> GetTurretTargetAngle(LimelightTarget::tValues target);

  /**
   * @brief Get the pitch and yaw of target
   *
   * @return LimelightTarget::tValues
   */
  LimelightTarget::tValues GetCameraTargetValues();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * @brief auto aims shooter to hub
   *
   */
  bool AutoAim();

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
   * @brief Get the speed of the shooter wheel in RPM
   *
   * @return units::revolutions_per_minute_t
   */
  units::revolutions_per_minute_t GetShooterWheelSpeed();

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
   * @brief Get the hood position in physical units
   *
   * @return hood position
   */
  units::degree_t GetHoodPosition();

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
   *
   * @note Turret can break itself if it moves above about 370 degrees or below about 10 degrees,
   *       but if turret starts near the 0/360 degree point, the homing position is ambiguous.
   *       To account for this, interpret position as greater than 360 if angle is in the 0-10/360-370
   *       degree region
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
   * @brief Gets the offset for the distance of targets above 7.416667 ft (89 in)
   *
   * @param actualDistance The actual distance of how far the camera is to the target
   * @return Gives the offset of the distance to subtract
   */
  units::inch_t GetPolynomialOffset(units::inch_t actualDistance);

  /**
   * @brief Get the shooter setpoints for a target distance
   *
   * @param distanceToTarget Distance from shooter to hub edge
   * @return Hood and shooter setpoints
   */
  ShooterDistanceSetpoints GetShooterDistanceSetpoints(units::inch_t distanceToTarget) const;

  /**
   * @brief Setting the shooter speed and hood angle depeneding on how far away the target is
   *
   * @param distanceToTarget The distance to the target from the robot
   * @return Hood and shooter setpoints
   */
  ShooterDistanceSetpoints SetShooterDistance(units::inch_t distanceToTarget);

  /**
   * @brief Handles fixed close shot shooting positions around hub
   *
   */
  void FixedShooterPosition(FixedPosState);

  /**
   * @brief Get the shooter speed in physical units
   *
   * @return Current shooter speed
   */
  units::angular_velocity::revolutions_per_minute_t GetShooterSpeed();

  /**
   * @brief Override automatic control
   */
  void ManualOverride();

  /**
   * @brief Set camera mode to driver mode without LEDs or targetting mode with LEDs
   *
   * @param driverMode True changes to driver mode, false changes to operator mode
   */
  void SetCameraDriverMode(bool driverMode);

  /**
   * @brief Checks if all shooter values are close enough to target values
   *
   * @param targets Desired values
   * @param real Current actual values
   * @return true if all real values close enough to targets, false otherwise
   */
  static bool InAcceptableRanges(const AimValues targets, const AimValues real);

 private:
  /**
   * @brief Stop vibration feedback
   */
  void StopFeedback() const;

  /**
   * @brief Activate vibration feedback to indicate aimed
   */
  void AimedFeedback() const;

  /**
   * @brief Generate hub-relative velocities for driving shot
   *
   * @param robotChassisSpeed Active robot velocity in chassis frame (x positive forward, y positive left, yaw positive CW)
   * @param hubTurretAngle Angle of hub in chassis reference frame (0 yaw is robot forward, positive is CW)
   * @return HubRelativeVelocities Robot's velocities relative to hub
   */
  HubRelativeVelocities ChassisVelocitiesToHubVelocities(const frc::ChassisSpeeds robotChassisSpeed,
                                                         units::degree_t hubTurretAngle);

  AimOffsets DrivingAimOffsets(const HubRelativeVelocities robotVelocity,
                               units::foot_t hubDistance,
                               units::degree_t hubAngle,
                               units::second_t targetStaleness);

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
  bool m_useCalculatedPitch;  ///< True if we want to use the calculated pitch to the top of the hood

  argos_lib::InterpolationMap<decltype(shooterRange::shooterSpeed.front().inVal),
                              shooterRange::shooterSpeed.size(),
                              decltype(shooterRange::shooterSpeed.front().outVal)>
      m_shooterSpeedMap;
  argos_lib::InterpolationMap<decltype(shooterRange::hoodAngle.front().inVal),
                              shooterRange::hoodAngle.size(),
                              decltype(shooterRange::hoodAngle.front().outVal)>
      m_hoodAngleMap;
  argos_lib::InterpolationMap<decltype(shooterRange::lateralSpeed.front().inVal),
                              shooterRange::lateralSpeed.size(),
                              decltype(shooterRange::lateralSpeed.front().outVal)>
      m_lateralSpeedMap;

  argos_lib::NTMotorPIDTuner m_hoodPIDTuner;
  argos_lib::NTMotorPIDTuner m_shooterPIDTuner;
  argos_lib::NTMotorPIDTuner m_turretPIDTuner;

  argos_lib::RobotInstance m_instance;
  SwerveDriveSubsystem* m_pDriveSubsystem;
  argos_lib::SwappableControllersSubsystem* m_pControllers;
};
