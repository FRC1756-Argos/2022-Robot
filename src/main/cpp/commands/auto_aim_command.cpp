/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_aim_command.h"

#include "argos_lib/general/debounce_settings.h"
#include "argos_lib/general/debouncer.h"
#include "argos_lib/general/swerve_utils.h"
#include "constants/field_points.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/math.h"

AutoAimCommand::AutoAimCommand(ShooterSubsystem* subsystem, SwerveDriveSubsystem* drive_subsystem)
    : m_shooter(subsystem)
    , m_drive(drive_subsystem)
    , m_threshDebounce({threshholds::shooter::acceptableRangeTime, 0_ms}) {
  if (subsystem != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void AutoAimCommand::Initialize() {
  if (m_shooter == nullptr) {
    Cancel();
    return;
  }
}

// Called repeatedly when this Command is scheduled to run
void AutoAimCommand::Execute() {
  if (m_shooter == nullptr) {
    Cancel();
    return;
  }

  frc::Pose2d curPos = m_drive->GetContinuousOdometry();

  // subract 24 inches because target is 4 feet Ø and vision aims from edge
  units::length::inch_t distanceToHub = units::length::inch_t(curPos.Translation().Distance(fieldPoints::hub)) - 24_in;

  units::length::inch_t a = fieldPoints::hub.X() - curPos.X();

  units::degree_t alpha = units::math::acos(a / distanceToHub);

  // change auto
  units::degree_t psi = argos_lib::swerve::ConstrainAngle(curPos.Rotation().Degrees(), 0_deg, 360_deg);

  units::degree_t turretAngle = alpha - psi;
}

// Called once the command ends or is interrupted.
void AutoAimCommand::End(bool interrupted) {
  if (m_shooter == nullptr) {
    return;
  }
}

// Returns true when the command should end.
bool AutoAimCommand::IsFinished() {
  if (m_shooter == nullptr) {
    // This function is finished when the target angle is reached

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
  const auto shooterSetpoints = m_shooter->GetShooterDistanceSetpoints(distanceToTarget);
  AimValues targets{turretTarget.value(), shooterSetpoints.hoodAngle, shooterSetpoints.shooterSpeed};

  AimValues real{turretReal.value(), m_shooter->GetHoodPosition(), m_shooter->GetShooterSpeed()};

  // DEBOUNCE
  if (m_threshDebounce(m_shooter->InAcceptableRanges(targets, real))) {
    return true;
  }

  return false;
}
