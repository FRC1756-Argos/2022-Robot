// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>


class SwerveModule
{
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


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void m_drive();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backRight;
  SwerveModule m_backLeft;


};
