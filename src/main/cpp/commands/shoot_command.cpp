/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/shoot_command.h"

ShootCommand::ShootCommand(IntakeSubsystem* subsystem) : m_pIntake(subsystem) {
  AddRequirements(m_pIntake);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShootCommand::Initialize() {
  m_pIntake->Shoot();
}

// Called repeatedly when this Command is scheduled to run
void ShootCommand::Execute() {}

// Called once the command ends or is interrupted.
void ShootCommand::End(bool interrupted) {
  m_pIntake->StopShoot();
}

// Returns true when the command should end.
bool ShootCommand::IsFinished() {
  bool ballPresent = m_pIntake->GetBallPresentShooter();
  if (m_previousBallPresent && !ballPresent) {
    m_previousBallPresent = ballPresent;
    return true;
  }
  m_previousBallPresent = ballPresent;
  return false;
}
