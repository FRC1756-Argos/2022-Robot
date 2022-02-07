/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>

#include "ctre/Phoenix.h"
#include "utils/file_system_homing_storage.h"
#include "utils/network_tables_wrapper.h"

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
};

class SwerveDriveSubsystem : public frc2::SubsystemBase {
 public:
  explicit SwerveDriveSubsystem(std::shared_ptr<NetworkTablesWrapper> networkTable);
  /**
   * @brief Main drive function for the robot
   *
   * @param fwVelocity Percent speed forward.  Range [-1.0, 1.0] where positive 1.0 is full speed forward
   * @param sideVelocity Percent speed perpendicular to the robots front.  Range [-1.0, 1.0] where positive 1.0 is full speed left
   * @param rotVelocity Percent speed of rotation of the chassis.  Range [-1.0, 1.0] where positive 1.0 is full speed counterclockwise
   */
  void SwerveDrive(const double& fwVelocity, const double& reVelocity, const double& rotVelocity);

  /**
   * @brief Save homes to persistent storage and updates module motors
   *
   * @param angle Current angle of all the swerve modules.  They should all be oriented the same during homing.
   *
   */
  void Home(const units::degree_t& angle);

  /**
   * @brief Initialize motors from persistent storage
   *
   */
  void InitializeMotors();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModule m_frontLeft;   ///< Front left swerve module
  SwerveModule m_frontRight;  ///< Front right swerve module
  SwerveModule m_backRight;   ///< Back right swerve module
  SwerveModule m_backLeft;    ///< Back left swerve module

  std::unique_ptr<frc::SwerveDriveKinematics<4>>
      m_pSwerveDriveKinematics;  ///< Kinematics model for swerve drive system

  // POINTER TO NETWORK TABLE CLASS OBJECT
  std::shared_ptr<NetworkTablesWrapper> m_pNetworkTable;  ///< Instance of network table class

  // std::FILE SYSTEM HOMING STORAGE
  FileSystemHomingStorage m_fsStorage;

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