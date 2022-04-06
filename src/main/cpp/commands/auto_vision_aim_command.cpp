/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_vision_aim_command.h"

#include "argos_lib/general/debounce_settings.h"
#include "argos_lib/general/debouncer.h"
#include "frc/smartdashboard/SmartDashboard.h"

AutoVisionAimCommand::AutoVisionAimCommand(ShooterSubsystem* subsystem)
    : m_pShooter(subsystem), m_threshDebounce({threshholds::shooter::acceptableRangeTime, 0_ms}) {
  if (subsystem != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void AutoVisionAimCommand::Initialize() {
  if (m_pShooter == nullptr) {
    Cancel();
    return;
  }
}

// Called repeatedly when this Command is scheduled to run
void AutoVisionAimCommand::Execute() {
  if (m_pShooter == nullptr) {
    Cancel();
    return;
  }

  m_pShooter->AutoAim(false);
}

// Called once the command ends or is interrupted.
void AutoVisionAimCommand::End(bool interrupted) {
  if (m_pShooter != nullptr) {
    m_pShooter->StopFeedback();
  }
}

// Returns true when the command should end.
bool AutoVisionAimCommand::IsFinished() {
  if (m_pShooter == nullptr) {
    return true;
  }

  return m_pShooter->AutoAim(false);
}
