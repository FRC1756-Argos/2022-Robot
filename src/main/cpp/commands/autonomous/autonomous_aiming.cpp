/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_aiming.h"

#include "constants/field_points.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
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

  units::degree_t turretAngle = GetTurretAngleToTarget(m_swerveDrive->GetContinuousOdometry(), fieldPoints::hub);

  // turret angle to smart dashboard in field reference frame.
  frc::SmartDashboard::PutNumber("(Static Auto Shoot) Turret Set Position", turretAngle.to<double>());

  // TODO: set angle based on the current auto position
  m_shooter->TurretSetPosition(turretAngle);
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

units::degree_t AutonomousAiming::GetTurretAngleToTarget(const frc::Pose2d curRobotPos, frc::Translation2d target) {
  units::degree_t curRobotRotation = curRobotPos.Rotation().Degrees();
  units::degree_t angleToTarget = GetAngleToPoint(curRobotPos.Translation(), target);
  return argos_lib::swerve::ConstrainAngle(360 - (curRobotRotation - angleToTarget), 0_deg, 360_deg);
}
