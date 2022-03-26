/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climb_command.h"

#include <queue>

#include "constants/climber_setpoints.h"

ClimbCommand::ClimbCommand(ClimberSubsystem* subsystem, std::vector<ClimberPoint> points)
    : m_pClimberSubsystem(subsystem), m_initPoints(points) {
  if (subsystem != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void ClimbCommand::Initialize() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }

  if (m_initPoints.empty()) {
    Cancel();
    return;
  }

  // TEMP
  if (m_initPoints.empty()) {
    Cancel();
    return;
  }

  for (ClimberPoint p : m_initPoints) {
    m_climbPoints.push(p);
  }
}

// Called repeatedly when this Command is scheduled to run
void ClimbCommand::Execute() {
  if (m_pClimberSubsystem == nullptr || m_pClimberSubsystem->IsManualOverride()) {
    Cancel();
    return;
  }

  ClimberPoint curPoint = m_climbPoints.front();
  if (m_pClimberSubsystem->ClimberAtPoint(curPoint)) {
    m_climbPoints.pop();
    return;
  } else {
    m_pClimberSubsystem->ClimberToSetpoint(curPoint);
  }
}

// Called once the command ends or is interrupted.
void ClimbCommand::End(bool interrupted) {
  if (m_pClimberSubsystem == nullptr) {
    return;
  }
  // Clear queue so no weird motion on next run
  m_climbPoints = {};
  m_pClimberSubsystem->SetClimbMotorsPID(0);  // Reset to unloaded
  m_pClimberSubsystem->Disable();
}

// Returns true when the command should end.
bool ClimbCommand::IsFinished() {
  if (m_pClimberSubsystem == nullptr) {
    return true;
  }

  // If no more points, we exit
  if (m_climbPoints.empty()) {
    return true;
  }

  return false;
}
