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

  // check if the robot is ready, if not, exit
  if (m_pClimberSubsystem->GetClimberStatus() != ClimberSubsystem::ClimberStatus::CLIMBER_READY) {
    Cancel();
    return;
  }

  for (ClimberPoint p : m_initPoints) {
    m_climbPoints.push(p);
  }
}

// Called repeatedly when this Command is scheduled to run
void ClimbCommand::Execute() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }

  if (m_pClimberSubsystem->GetClimberStatus() != ClimberSubsystem::ClimberStatus::CLIMBER_READY) {
    Cancel();
    return;
  }

  /// @todo did the user disable the climb from the controller?
  ClimberPoint curPoint = m_climbPoints.front();
  if (m_pClimberSubsystem->ClimberAtPoint(curPoint)) {
    m_climbPoints.pop();
    return;
  }
  if (!m_pClimberSubsystem->ClimberAtPoint(curPoint)) {
    m_pClimberSubsystem->ClimberToSetpoint(curPoint);
  }
}

// Called once the command ends or is interrupted.
void ClimbCommand::End(bool interrupted) {
  if (m_pClimberSubsystem == nullptr) {
    return;
  }
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
