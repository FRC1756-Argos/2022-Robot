/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_hood_command.h"

#include "units/time.h"

HomeHoodCommand::HomeHoodCommand(ShooterSubsystem* shooter)
    : m_pShooter(shooter), m_hoodMovingDebounce{{0_ms, 500_ms}, true} {
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void HomeHoodCommand::Initialize() {
  m_pShooter->MoveHood(0.23);
}

// Called repeatedly when this Command is scheduled to run
void HomeHoodCommand::Execute() {
  if (m_pShooter->IsManualOverride()) {
    Cancel();
  } else {
    m_pShooter->MoveHood(0.23);
  }
}

// Called once the command ends or is interrupted.
void HomeHoodCommand::End(bool interrupted) {
  if (!interrupted) {
    m_pShooter->UpdateHoodHome();
    std::printf("Save home!\n");
  }
  m_pShooter->MoveHood(0.0);
}

// Returns true when the command should end.
bool HomeHoodCommand::IsFinished() {
  return !m_hoodMovingDebounce(m_pShooter->IsHoodMoving());
}
