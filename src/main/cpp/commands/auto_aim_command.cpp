/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_aim_command.h"

AutoAimCommand::AutoAimCommand(ShooterSubsystem* subsystem,
                               argos_lib::InterpolationMap<units::length::inch_t, 6, units::angle::degree_t>* hoodMap)
    : m_shooter(subsystem), m_hoodMap(hoodMap) {
  if (subsystem != nullptr || m_hoodMap != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void AutoAimCommand::Initialize() {
  if (m_shooter == nullptr || m_hoodMap == nullptr) {
    Cancel();
    return;
  }
}

// Called repeatedly when this Command is scheduled to run
void AutoAimCommand::Execute() {
  if (m_shooter == nullptr || m_hoodMap == nullptr) {
    Cancel();
    return;
  }

  m_shooter->AutoAim();
}

// Called once the command ends or is interrupted.
void AutoAimCommand::End(bool interrupted) {
  if (m_shooter == nullptr || m_hoodMap == nullptr) {
    return;
  }
}

// Returns true when the command should end.
bool AutoAimCommand::IsFinished() {
  if (m_shooter == nullptr || m_hoodMap == nullptr) {
    return true;
  }

  // Get current target
  LimelightTarget::tValues cameraTarget = m_shooter->GetCameraTargetValues();

  // Get turret target & real
  std::optional<units::degree_t> turretTarget = m_shooter->GetTurretTargetAngle(cameraTarget);
  std::optional<units::degree_t> turretReal = m_shooter->TurretGetPosition();

  // Get hood target & real
  units::inch_t distanceToTarget = m_shooter->GetTargetDistance(cameraTarget.pitch);
  units::degree_t hoodReal = m_shooter->GetHoodPosition();
  units::degree_t hoodTarget = m_hoodMap->Map(distanceToTarget);

  // Get shooter speed target & real

  if (!turretReal) {
    return false;
  }

  return false;
}

template <typename T>
bool AutoAimCommand::InThreshold(T value, T threshold) {
  if (value < value - threshold || value > value + threshold) {
    return false;
  } else {
    return true;
  }
}
