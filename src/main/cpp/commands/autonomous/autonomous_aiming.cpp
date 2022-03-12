/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_aiming.h"

AutonomousAiming::AutonomousAiming(ShooterSubsystem* subsystem)
    : m_shooter(subsystem), m_threshDebounce({threshholds::shooter::acceptableRangeTime, 0_ms}) {
  if (subsystem != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void AutonomousAiming::Initialize() {
  if (m_shooter == nullptr) {
    Cancel();
    return;
  }
}

// Called repeatedly when this Command is scheduled to run
void AutonomousAiming::Execute() {
  if (m_shooter == nullptr) {
    Cancel();
    return;
  }

  // TODO: set angle based on the current auto position
  m_shooter->TurretSetPosition(170_deg);
}

// Called once the command ends or is interrupted.
void AutonomousAiming::End(bool interrupted) {}

// Returns true when the command should end.
bool AutonomousAiming::IsFinished() {
  return true;
}
