/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <commands/auto_position_aim_command.h>
#include <commands/auto_vision_aim_command.h>
#include <commands/delay_command.h>
#include <commands/drive_to_location.h>
#include <commands/intake_command.h>
#include <commands/shoot_command.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/shooter_subsystem.h>
#include <subsystems/swerve_drive_subsystem.h>

#include <string>

#include "autonomous_command.h"
#include "autonomous_right_2ball.h"

class AutonomousRight5ballGap
    : public frc2::CommandHelper<frc2::CommandBase, AutonomousRight5ballGap>
    , public AutonomousCommand {
 public:
  AutonomousRight5ballGap(IntakeSubsystem* pIntake, ShooterSubsystem* pShooter, SwerveDriveSubsystem* pDrive);

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

  // AutonomousRight2ball m_2ball;
  DelayCommand m_humanPlayerDelay;
  DelayCommand m_shootDelay;
  DriveToLocation m_driveToBallB;
  DriveToLocation m_driveToBallD;
  DriveToLocation m_driveBackFromTerminal;
  DriveToLocation m_driveToFinalShootPosition;
  ShootCommand m_shootOneBall;
  ShootCommand m_shootTwoBall;
  AutoPositionAimCommand m_aimBallB;
  AutoPositionAimCommand m_aimBallD;
  AutoVisionAimCommand m_visionAim;

  frc2::SequentialCommandGroup m_allCommands;
};
