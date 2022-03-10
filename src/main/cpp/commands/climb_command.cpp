/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climb_command.h"

climb_command::climb_command(ClimberSubsystem* subsystem) : m_pClimberSubsystem(subsystem) {
  if (subsystem != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void climb_command::Initialize() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }
}

// Called repeatedly when this Command is scheduled to run
void climb_command::Execute() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }
}

// Called once the command ends or is interrupted.
void climb_command::End(bool interrupted) {
  if (m_pClimberSubsystem == nullptr) {
    return;
  }
}

// Returns true when the command should end.
bool climb_command::IsFinished() {
  if (m_pClimberSubsystem == nullptr) {
    return true;
  }
  return false;
}
