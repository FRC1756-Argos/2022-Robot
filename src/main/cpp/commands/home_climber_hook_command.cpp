/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_climber_hook_command.h"

using namespace std::chrono_literals;

HomeClimberHookCommand::HomeClimberHookCommand(ClimberSubsystem* subsystem)
    : m_pClimberSubsystem{subsystem}, m_hookMovingDebounce{{0_ms, 500_ms}, true} {
  if (m_pClimberSubsystem != nullptr) {
    AddRequirements(m_pClimberSubsystem);
  }
}

// Called when the command is initially scheduled.
void HomeClimberHookCommand::Initialize() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }
  m_pClimberSubsystem->MoveHook(0.07);
  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void HomeClimberHookCommand::Execute() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }
  if (m_pClimberSubsystem->IsManualOverride() || (std::chrono::steady_clock::now() - m_startTime) > 2.0s) {
    Cancel();
  } else {
    m_pClimberSubsystem->MoveHook(0.07);
  }
}

// Called once the command ends or is interrupted.
void HomeClimberHookCommand::End(bool interrupted) {
  if (m_pClimberSubsystem == nullptr) {
    return;
  }
  if (!interrupted) {
    m_pClimberSubsystem->UpdateHookHome();
  }
  m_pClimberSubsystem->MoveHook(0.0);
}

// Returns true when the command should end.
bool HomeClimberHookCommand::IsFinished() {
  if (m_pClimberSubsystem == nullptr) {
    return true;
  }
  return !m_hookMovingDebounce(m_pClimberSubsystem->IsHookMoving());
}
