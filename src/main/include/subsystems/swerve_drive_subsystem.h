/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/ADIS16448_IMU.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>

#include "argos_lib/config/robot_instance.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "ctre/Phoenix.h"
#include "utils/file_system_homing_storage.h"
#include "utils/network_tables_wrapper.h"
#include "utils/swerve_trapezoidal_profile.h"

class SwerveModule {
 public:
  // MOTORS
  WPI_TalonFX m_drive;
  WPI_TalonFX m_turn;
  // ENCODER
  CANCoder m_encoder;

  /**
   * @brief Construct a new Swerve Module object
   *
   * @param driveAddr address of the drive motor on the module
   * @param turnAddr address of the turn motor on the module
   * @param encoderAddr address of the encoder on this module
   */
  SwerveModule(const char driveAddr, const char turnAddr, const char encoderAddr);

  frc::SwerveModuleState GetState();
};

class SwerveDriveSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * @brief Enumerator for either field-centric or robot centric drive modes.
   *
   */
  enum DriveControlMode { fieldCentricControl, robotCentricControl };

  explicit SwerveDriveSubsystem(std::shared_ptr<NetworkTablesWrapper> networkTable,
                                const argos_lib::RobotInstance instance);

  /**
   * @brief Handle the robot disabling
   */
  void Disable();

  /**
   * @brief Main drive function for the robot
   *
   * @param fwVelocity Percent speed forward.  Range [-1.0, 1.0] where positive 1.0 is full speed forward
   * @param sideVelocity Percent speed perpendicular to the robots front.  Range [-1.0, 1.0] where positive 1.0 is full speed left
   * @param rotVelocity Percent speed of rotation of the chassis.  Range [-1.0, 1.0] where positive 1.0 is full speed counterclockwise
   */
  void SwerveDrive(const double& fwVelocity, const double& sideVelocity, const double& rotVelocity);

  /**
   * @brief Stop all motors
   */
  void StopDrive();

  /**
   * @brief Save homes to persistent storage and updates module motors
   *
   * @param angle Current angle of all the swerve modules.  They should all be oriented the same during homing.
   *
   */
  void Home(const units::degree_t& angle);

  /**
   * @brief Tell the robot it's in it's correct field-oriented "Front"
   *
   * @param homeAngle Current orientation of the robot
   * @param updateOdometry Also update odometry field-centric angle
   */
  void FieldHome(units::degree_t homeAngle = 0_deg, bool updateOdometry = true);

  /**
   * @brief Set current robot position.  Useful for initializing initial position before autonomous
   *
   * @param currentPose Field-centric pose of the robot
   */
  void InitializeOdometry(const frc::Pose2d& currentPose);

  /**
   * @brief Reads module states & gyro, updates odometry, and returns latest pose estimate
   *
   * @return Estimate of robot pose
   */
  frc::Pose2d UpdateOdometry();

  /**
   * @brief Get the field-centric angle of the robot based on gyro and saved reference orientation
   *
   * @return Field-centric angle of the robot where 0 degrees is intake oriented toward
   *         opposing alliance operator station positive CCW.
   */
  units::degree_t GetFieldCentricAngle() const;

  /**
   * @brief Get the latest pose estimate
   *
   * @return Latest pose
   */
  frc::Pose2d GetPoseEstimate() const;

  void SetControlMode(SwerveDriveSubsystem::DriveControlMode controlMode);

  /**
   * @brief Initialize motors from persistent storage
   *
   */
  void InitializeMotors();

  /// @todo implement these
  void UpdateFollowerLinearPIDParams(double kP, double kI, double kD);
  void UpdateFollowerRotationalPIDParams(double kP, double kI, double kD);
  void UpdateFollowerRotationalPIDConstraints(frc::TrapezoidProfile<units::degrees>::Constraints constraints);

  void StartDrivingProfile(SwerveTrapezoidalProfileSegment newProfile);

  void CancelDrivingProfile();

 private:
  DriveControlMode m_controlMode;  ///< Active control mode

  SwerveModule m_frontLeft;   ///< Front left swerve module
  SwerveModule m_frontRight;  ///< Front right swerve module
  SwerveModule m_backRight;   ///< Back right swerve module
  SwerveModule m_backLeft;    ///< Back left swerve module

  // GYROSCOPIC SENSORS
  frc::ADIS16448_IMU m_imu;

  units::degree_t m_fieldHomeOffset;

  /**
 * @brief A struct for holding the 3 different input velocities, for organization
 *
 */
  struct Velocities {
    const double fwVelocity;
    const double sideVelocity;
    const double rotVelocity;
  };

  frc::SwerveDriveKinematics<4> m_swerveDriveKinematics;  ///< Kinematics model for swerve drive system

  frc::SwerveDriveOdometry<4> m_odometry;  ///< Odometry to track robot

  // POINTER TO NETWORK TABLE CLASS OBJECT
  std::shared_ptr<NetworkTablesWrapper> m_pNetworkTable;  ///< Instance of network table class

  // std::FILE SYSTEM HOMING STORAGE
  FileSystemHomingStorage m_fsStorage;

  bool m_followingProfile;
  std::unique_ptr<SwerveTrapezoidalProfileSegment> m_pActiveSwerveProfile;
  std::chrono::time_point<std::chrono::steady_clock> m_swerveProfileStartTime;
  frc2::PIDController m_linearPID;
  frc::ProfiledPIDController<units::radians> m_rotationalPID;
  frc::HolonomicDriveController m_followerController;

  argos_lib::NTMotorPIDTuner m_driveMotorPIDTuner;

  /**
 * @brief Get the Raw Module States object and switch between robot-centric and field-centric
 *
 * @param velocities Desired velocity
 * @return wpi::array<frc::SwerveModuleState, 4>
 */
  wpi::array<frc::SwerveModuleState, 4> GetRawModuleStates(SwerveDriveSubsystem::Velocities velocities);

  /**
   * @brief Get the active states of all swerve modules
   *
   * @return Active module states
   */
  wpi::array<frc::SwerveModuleState, 4> GetCurrentModuleStates();

  /**
   * @brief HomeToNetworkTables all of the modules back to zero
   *
   * @param angle the angle of the module in it's current state
   */
  void HomeToNetworkTables(const units::degree_t& angle);

  /**
   * @brief Will load saved homes, and set the encoders to reset to true angle relative to robot "front"
   *
   */
  void InitializeMotorsFromNetworkTables();

  /**
   * @brief Save homes to a file
   *
   * @param angle Current angle of all the swerve modules.  They should all be oriented the same during homing.
   *
   */
  void HomeToFS(const units::degree_t& angle);

  /**
   * @brief Initialize motors from saved file
   *
   */
  void InitializeMotorsFromFS();
};
