// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot_command.h"

Shoot_command::Shoot_command(IntakeSubsystem* subsystem):m_shooter(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Shoot_command::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Shoot_command::Execute() {}

// Called once the command ends or is interrupted.
void Shoot_command::End(bool interrupted) {}

// Returns true when the command should end.
bool Shoot_command::IsFinished() {
  return false;
}
