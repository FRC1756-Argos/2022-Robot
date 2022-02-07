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
  m_shooter->ManualAim(0.0, -0.1);
}

// Called repeatedly when this Command is scheduled to run
void HomeHoodCommand::Execute() {
  m_shooter->ManualAim(0.0, -0.1);
}

// Called once the command ends or is interrupted.
void HomeHoodCommand::End(bool interrupted) {
  if (!interrupted) {
    m_shooter->UpdateHoodHome();
  }
}

// Returns true when the command should end.
bool HomeHoodCommand::IsFinished() {
  return m_hoodMovingDebounce(m_shooter->IsHoodMoving());
}
