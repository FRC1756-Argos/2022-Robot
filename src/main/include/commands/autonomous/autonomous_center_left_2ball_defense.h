/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <commands/auto_position_aim_command.h>
#include <commands/auto_vision_aim_command.h>
#include <commands/delay_command.h>
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

#include <string>

#include "autonomous_command.h"

class AutonomousCenterLeft2ballDefense
    : public frc2::CommandHelper<frc2::CommandBase, AutonomousCenterLeft2ballDefense>
    , public AutonomousCommand {
 public:
  AutonomousCenterLeft2ballDefense(IntakeSubsystem* pIntake, ShooterSubsystem* pShooter, SwerveDriveSubsystem* pDrive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::Command* GetCommand() final;

 private:
  IntakeSubsystem* m_pIntake;
  ShooterSubsystem* m_pShooter;
  SwerveDriveSubsystem* m_pDrive;

  DelayCommand m_startDelay;
  DriveToLocation m_driveToBallC;
  DriveToLocation m_driveDefenseRight;
  DriveToLocation m_driveDefenseLeft;
  HomeHoodCommand m_homeHoodCommand;
  IntakeCommand m_intakeCommand;
  ShootCommand m_shootCommand;
  AutoPositionAimCommand m_aimBallC;
  AutoPositionAimCommand m_aimDefenseBottom;
  AutoVisionAimCommand m_visionAim;
  InitializeOdometryCommand m_initOdometry;

  frc2::SequentialCommandGroup m_allCommands;
};
