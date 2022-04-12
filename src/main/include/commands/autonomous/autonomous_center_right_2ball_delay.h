/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once


#include <commands/delay_command.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/intake_subsystem.h>
#include <subsystems/shooter_subsystem.h>
#include <subsystems/swerve_drive_subsystem.h>

#include <string>

#include "autonomous_command.h"
#include "autonomous_center_right_2ball.h"
class AutonomousCenterRight2ballDelay
    : public frc2::CommandHelper<frc2::CommandBase, AutonomousCenterRight2ballDelay>
    , public AutonomousCommand {
 public:
  AutonomousCenterRight2ballDelay(IntakeSubsystem* pIntake, ShooterSubsystem* pShooter, SwerveDriveSubsystem* pDrive);

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


  frc2::SequentialCommandGroup m_allCommands;
};
