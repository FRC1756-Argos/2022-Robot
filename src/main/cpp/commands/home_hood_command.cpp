/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_hood_command.h"

#include "units/time.h"

HomeHoodCommand::HomeHoodCommand(ShooterSubsystem* shooter) : m_shooter(shooter), m_hoodMovingDebounce{{500_ms, 0_ms}} {
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void HomeHoodCommand::Initialize() {
  // @todo same as Execute()
}

// Called repeatedly when this Command is scheduled to run
void HomeHoodCommand::Execute() {
  /// @todo Slowly move hood toward home position
}

// Called once the command ends or is interrupted.
void HomeHoodCommand::End(bool interrupted) {
  /// @todo Save home if ends normally (not interrupted)
}

// Returns true when the command should end.
bool HomeHoodCommand::IsFinished() {
  /// @todo Return true when motor in home position (use debounced IsHoodMoving)
  return false;
}
