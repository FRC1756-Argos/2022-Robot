/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_right_2ball_delay.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousRight2ballDelay::AutonomousRight2ballDelay(IntakeSubsystem* pIntake,
                                             ShooterSubsystem* pShooter,
                                             SwerveDriveSubsystem* pDrive)
    : m_pIntake{pIntake}
    , m_pShooter{pShooter}
    , m_pDrive{pDrive}
    , m_startDelay{5_s}
    , m_allCommands{} {
  m_allCommands.AddCommands(
      m_startDelay, AutonomousRight2ballDelay{m_pIntake, m_pShooter, m_pDrive});

}

// Called when the command is initially scheduled.
void AutonomousRight2ballDelay::Initialize() {
  m_pShooter->SetCameraDriverMode(false);
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousRight2ballDelay::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousRight2ballDelay::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousRight2ballDelay::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousRight2ballDelay::GetName() const {
  return "Right 2 Ball with delay";
}

frc2::Command* AutonomousRight2ballDelay::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
