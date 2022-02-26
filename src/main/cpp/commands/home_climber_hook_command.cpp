/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_climber_hook_command.h"

HomeClimberHookCommand::HomeClimberHookCommand(ClimberSubsystem* subsystem)
    : m_pClimberSubsystem{subsystem}, m_hookMovingDebounce{{0_ms, 500_ms}, true} {
  AddRequirements(m_pClimberSubsystem);
}

// Called when the command is initially scheduled.
void HomeClimberHookCommand::Initialize() {
  m_pClimberSubsystem->MoveHook(0.07);
}

// Called repeatedly when this Command is scheduled to run
void HomeClimberHookCommand::Execute() {
  if (m_pClimberSubsystem->IsManualOverride()) {
    Cancel();
  } else {
    m_pClimberSubsystem->MoveHook(0.07);
  }
}

// Called once the command ends or is interrupted.
void HomeClimberHookCommand::End(bool interrupted) {
  if (!interrupted) {
    m_pClimberSubsystem->UpdateHookHome();
  }
  m_pClimberSubsystem->MoveHook(0.0);
}

// Returns true when the command should end.
bool HomeClimberHookCommand::IsFinished() {
  return !m_hookMovingDebounce(m_pClimberSubsystem->IsHookMoving());
}
