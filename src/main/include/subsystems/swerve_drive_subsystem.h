/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"
#include <memory>

class SwerveModule {
 public:
  // MOTORS
  WPI_TalonFX m_drive;
  WPI_TalonFX m_turn;
  // ENCODER
  CANCoder m_encoder;

  SwerveModule(const char driveAddr, const char turnAddr, const char encoderAddr);
};

class SwerveDriveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveDriveSubsystem();
  void m_drive(const double &fwVelocity, const double &reVelocity, const double &rotVelocity);
  std::unique_ptr<frc::SwerveDriveKinematics<4>> m_pSwerveDriveKinematics;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backRight;
  SwerveModule m_backLeft;
};
