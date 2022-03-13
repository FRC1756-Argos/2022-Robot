/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/intake_command.h"

IntakeCommand::IntakeCommand(IntakeSubsystem* subsystem) : m_pIntake(subsystem) {
  AddRequirements(m_pIntake);

  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IntakeCommand::Initialize() {
  m_pIntake->Intake();
}

// Called repeatedly when this Command is scheduled to run
void IntakeCommand::Execute() {}

// Called once the command ends or is interrupted.
void IntakeCommand::End(bool interrupted) {
  m_pIntake->StopIntake();
}

// Returns true when the command should end.
bool IntakeCommand::IsFinished() {
  bool ballPresent = m_pIntake->GetBallPresentIntake();
  if (!m_previousBallPresent && ballPresent) {
    m_previousBallPresent = ballPresent;
    return true;
  }
  m_previousBallPresent = ballPresent;
  return false;
}
