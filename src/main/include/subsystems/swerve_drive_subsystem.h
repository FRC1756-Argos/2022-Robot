/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>

#include "argos_lib/general/swerve_utils.h"
#include "ctre/Phoenix.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

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
  SwerveDriveSubsystem();
  /**
 * @brief Main drive function for the robot
 *
 * @param fwVelocity velocity in meters per second forward
 * @param sideVelocity velocity in meters per second perpendicular to the robots front
 * @param rotVelocity velocity in radians per second of rotation of the chasis
 */
  void SwerveDrive(const double& fwVelocity, const double& reVelocity, const double& rotVelocity);

  /**
 * @brief Home all of the modules back to zero
 *
 * @param angle the angle of the module in it's current state
 */
  void Home(const units::degree_t& angle);

  /**
   * @brief Will load saved homes, and set the encoders to reset to true angle relative to robot "front"
   *
   */
  void InitializeMotors();

  std::unique_ptr<frc::SwerveDriveKinematics<4>> m_pSwerveDriveKinematics;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModule m_frontLeft;   //< Front left swerve module
  SwerveModule m_frontRight;  //< Front right swerve module
  SwerveModule m_backRight;   //< Back right swerve module
  SwerveModule m_backLeft;    //< Back left swerve module

  // NETWORK TABLE INSTANCE & ENTRIES
  nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();  //< Auto-Created network table instance

  // NETWORK TABLES
  std::shared_ptr<nt::NetworkTable> m_homeTable = ntInstance.GetTable("argos/motorHomes");

  // ENTRIES
  nt::NetworkTableEntry m_ntFrontLeft =
      m_homeTable->GetEntry("frontLeft");  //< Absolute encoder value for front left home that denotes "forward"
  nt::NetworkTableEntry m_ntFrontRight =
      m_homeTable->GetEntry("frontRight");  //< Absolute encoder value for front right home that denotes "forward"
  nt::NetworkTableEntry m_ntBackRight =
      m_homeTable->GetEntry("backRight");  //< Absolute encoder value for back right home that denotes "forward"
  nt::NetworkTableEntry m_ntBackLeft =
      m_homeTable->GetEntry("backLeft");  //< Absolute encoder value for back left home that denotes "forward"
};
