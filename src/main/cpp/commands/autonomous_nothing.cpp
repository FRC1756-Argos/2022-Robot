/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous_nothing.h"

autonomous_nothing::autonomous_nothing() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void autonomous_nothing::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void autonomous_nothing::Execute() {}

// Called once the command ends or is interrupted.
void autonomous_nothing::End(bool interrupted) {}

// Returns true when the command should end.
bool autonomous_nothing::IsFinished() {
  return true;
}
