/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_aiming.h"

#include "frc/geometry/Pose2d.h"
#include "units/math.h"

AutonomousAiming::AutonomousAiming(ShooterSubsystem* shootSys, SwerveDriveSubsystem* driveSys)
    : m_shooter(shootSys)
    , m_swerveDrive(driveSys)
    , m_threshDebounce({threshholds::shooter::acceptableRangeTime, 0_ms}) {
  if (shootSys != nullptr) {
    AddRequirements(shootSys);
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

  frc::Pose2d curPos = m_swerveDrive->GetContinuousOdometry();

  // TODO: set angle based on the current auto position
  m_shooter->TurretSetPosition(170_deg);
}

// Called once the command ends or is interrupted.
void AutonomousAiming::End(bool interrupted) {}

// Returns true when the command should end.
bool AutonomousAiming::IsFinished() {
  return true;
}

// ========================================= HELPERS =========================================

units::degree_t AutonomousAiming::GetOffsetAngle(frc::Translation2d curPos, frc::Translation2d target) {
  if (curPos.X() < target.X() && curPos.Y() < target.Y()) {
    // quadrant 1
    return 0_deg;
  } else if (curPos.X() > target.X() && curPos.Y() < target.Y()) {
    // quadrant 2
    return -180_deg;
  } else if (curPos.X() > target.X() && curPos.Y() > target.Y()) {
    // quadrant 3
    return 180_deg;
  } else if (curPos.X() < target.X() && curPos.Y() > target.Y()) {
    // quadrant 4
    return -360_deg;
  }
}

units::degree_t AutonomousAiming::GetAngleToPoint(frc::Translation2d curPos, frc::Translation2d target) {
  units::degree_t offset = GetOffsetAngle(curPos, target);
  units::degree_t psi = units::math::acos((units::math::abs(target.X() - curPos.X()) / curPos.Distance(target)));
  units::degree_t finalAngle;
  if (offset < 0_deg) {
    return units::math::abs(offset + psi);
  } else {
    return offset + psi;
  }
}
