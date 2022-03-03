/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_aim_command.h"

#include "argos_lib/general/debounce_settings.h"
#include "argos_lib/general/debouncer.h"
#include "frc/smartdashboard/SmartDashboard.h"

AutoAimCommand::AutoAimCommand(
    ShooterSubsystem* subsystem,
    argos_lib::InterpolationMap<units::length::inch_t, 6, units::angle::degree_t>* hoodMap,
    argos_lib::InterpolationMap<units::length::inch_t, 6, units::angular_velocity::revolutions_per_minute_t>* wheelMap)
    : m_shooter(subsystem)
    , m_hoodMap(hoodMap)
    , m_wheelMap(wheelMap)
    , m_threshDebounce({threshholds::shooter::acceptableRangeTime, 0_ms}) {
  if (subsystem != nullptr || m_hoodMap != nullptr || m_wheelMap != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void AutoAimCommand::Initialize() {
  if (m_shooter == nullptr || m_hoodMap == nullptr || m_wheelMap == nullptr) {
    Cancel();
    return;
  }
}

// Called repeatedly when this Command is scheduled to run
void AutoAimCommand::Execute() {
  if (m_shooter == nullptr || m_hoodMap == nullptr || m_wheelMap == nullptr) {
    Cancel();
    return;
  }

  m_shooter->AutoAim();
}

// Called once the command ends or is interrupted.
void AutoAimCommand::End(bool interrupted) {
  if (m_shooter == nullptr || m_hoodMap == nullptr || m_wheelMap == nullptr) {
    return;
  }
}

// Returns true when the command should end.
bool AutoAimCommand::IsFinished() {
  if (m_shooter == nullptr || m_hoodMap == nullptr || m_wheelMap == nullptr) {
    return true;
  }

  // Get current target
  LimelightTarget::tValues cameraTarget = m_shooter->GetCameraTargetValues();

  // Get turret target & real
  std::optional<units::degree_t> turretTarget = m_shooter->GetTurretTargetAngle(cameraTarget);
  std::optional<units::degree_t> turretReal = m_shooter->TurretGetPosition();

  if (!turretReal || !turretTarget) {
    return false;
  }

  // Get hood target & real
  units::inch_t distanceToTarget = m_shooter->GetTargetDistance(cameraTarget.pitch);

  // Get all targets:
  AutoAimCommand::aimValues targets{
      turretTarget.value(), m_hoodMap->Map(distanceToTarget), m_wheelMap->Map(distanceToTarget)};

  AutoAimCommand::aimValues real{turretReal.value(), m_shooter->GetHoodPosition(), m_shooter->GetShooterSpeed()};

  // DEBOUNCE
  if (m_threshDebounce(InAcceptableRanges(targets, real))) {
    return true;
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

bool AutoAimCommand::InAcceptableRanges(AutoAimCommand::aimValues targets, AutoAimCommand::aimValues real) {
  if (InThreshold<units::degree_t>(targets.turretTarget, threshholds::shooter::acceptableTurretError)) {
    frc::SmartDashboard::PutBoolean("Acceptable Error) Turret", true);
    if (InThreshold<units::degree_t>(targets.hoodTarget, threshholds::shooter::acceptableHoodError)) {
      frc::SmartDashboard::PutBoolean("(Acceptable Error) Hood", true);
      if (InThreshold<units::angular_velocity::revolutions_per_minute_t>(targets.shooterTarget,
                                                                         threshholds::shooter::acceptableWheelError)) {
        frc::SmartDashboard::PutBoolean("(Acceptable Error) Wheel Speed", true);
        return true;
      } else {
        frc::SmartDashboard::PutBoolean("(Acceptable Error) Wheel Speed", false);
        return false;
      }
    } else {
      frc::SmartDashboard::PutBoolean("(Acceptable Error) Hood", false);
      return false;
    }
  } else {
    frc::SmartDashboard::PutBoolean("Acceptable Error) Turret", false);
    return false;
  }
}
