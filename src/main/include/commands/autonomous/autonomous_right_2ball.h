/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <commands/auto_position_aim_command.h>
#include <commands/drive_to_location.h>
#include <commands/home_hood_command.h>
#include <commands/initialize_odometry_command.h>
#include <commands/intake_command.h>
#include <commands/shoot_command.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/intake_subsystem.h>
#include <subsystems/shooter_subsystem.h>
#include <subsystems/swerve_drive_subsystem.h>

class AutonomousRight2ball : public frc2::CommandHelper<frc2::CommandBase, AutonomousRight2ball> {
 public:
  AutonomousRight2ball(IntakeSubsystem* pIntake, ShooterSubsystem* pShooter, SwerveDriveSubsystem* pDrive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem* m_pIntake;
  ShooterSubsystem* m_pShooter;
  SwerveDriveSubsystem* m_pDrive;

  DriveToLocation m_driveToBallA;
  HomeHoodCommand m_homeHoodCommand;
  IntakeCommand m_intakeCommand;
  ShootCommand m_shootCommand;
  AutoPositionAimCommand m_aimBallA;
  InitializeOdometryCommand m_initOdometry;

  frc2::SequentialCommandGroup m_allCommands;
};
