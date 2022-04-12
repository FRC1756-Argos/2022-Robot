/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_center_1ball_delay.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousCenter1ballDelay::AutonomousCenter1ballDelay(IntakeSubsystem* pIntake,
                                             ShooterSubsystem* pShooter,
                                             SwerveDriveSubsystem* pDrive)
    : m_pIntake{pIntake}
    , m_pShooter{pShooter}
    , m_pDrive{pDrive}
    , m_startDelay{5_s}
    , m_allCommands{} {
  m_allCommands.AddCommands(
      m_startDelay, AutonomousCenter1ball{m_pIntake, m_pShooter, m_pDrive});

}

// Called when the command is initially scheduled.
void AutonomousCenter1ballDelay::Initialize() {
  m_pShooter->SetCameraDriverMode(false);
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCenter1ballDelay::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCenter1ballDelay::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCenter1ballDelay::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousCenter1ballDelay::GetName() const {
  return "Center 1 Ball with delay";
}

frc2::Command* AutonomousCenter1ballDelay::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
