/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_hood_command.h"

#include "units/time.h"

using namespace std::chrono_literals;

HomeHoodCommand::HomeHoodCommand(ShooterSubsystem* shooter)
    : m_pShooter(shooter), m_hoodMovingDebounce{{0_ms, 500_ms}, true} {
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void HomeHoodCommand::Initialize() {
  m_startTime = std::chrono::steady_clock::now();
  m_pShooter->MoveHood(0.2);
}

// Called repeatedly when this Command is scheduled to run
void HomeHoodCommand::Execute() {
  if (m_pShooter->IsManualOverride() || (std::chrono::steady_clock::now() - m_startTime) > 2.0s) {
    Cancel();
  } else {
    m_pShooter->MoveHood(0.2);
  }
}

// Called once the command ends or is interrupted.
void HomeHoodCommand::End(bool interrupted) {
  if (!interrupted) {
    m_pShooter->UpdateHoodHome();
  }
  m_pShooter->MoveHood(0.0);
}

// Returns true when the command should end.
bool HomeHoodCommand::IsFinished() {
  return !m_hoodMovingDebounce(m_pShooter->IsHoodMoving());
}
