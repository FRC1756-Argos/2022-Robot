/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_position_aim_command.h"

#include "constants/field_points.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/math.h"

AutoPositionAimCommand::AutoPositionAimCommand(ShooterSubsystem* shootSys, frc::Pose2d shootingPosition)
    : m_pShooter(shootSys)
    , m_shootingPosition(shootingPosition)
    , m_threshDebounce({threshholds::shooter::acceptableRangeTime, 0_ms}) {
  if (shootSys != nullptr) {
    AddRequirements(shootSys);
  }
}

// Called when the command is initially scheduled.
void AutoPositionAimCommand::Initialize() {
  if (m_pShooter == nullptr) {
    Cancel();
    return;
  }
  units::degree_t turretAngle = GetTurretAngleToTarget(m_shootingPosition, fieldPoints::hub);

  // turret angle to smart dashboard in field reference frame.
  frc::SmartDashboard::PutNumber("(Static Auto Shoot) Turret Set Position", turretAngle.to<double>());

  m_pShooter->TurretSetPosition(turretAngle);
}

// Called repeatedly when this Command is scheduled to run
void AutoPositionAimCommand::Execute() {
  if (m_pShooter == nullptr) {
    Cancel();
    return;
  }
}

// Called once the command ends or is interrupted.
void AutoPositionAimCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoPositionAimCommand::IsFinished() {
  return true;
}

// ========================================= HELPERS =========================================

units::degree_t AutoPositionAimCommand::GetOffsetAngle(frc::Translation2d curPos, frc::Translation2d target) {
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
  return 0_deg;
}

units::degree_t AutoPositionAimCommand::GetAngleToPoint(frc::Translation2d curPos, frc::Translation2d target) {
  units::degree_t offset = GetOffsetAngle(curPos, target);
  units::degree_t psi = units::math::acos((units::math::abs(target.X() - curPos.X()) / curPos.Distance(target)));
  units::degree_t finalAngle;
  if (offset < 0_deg) {
    return units::math::abs(offset + psi);
  } else {
    return offset + psi;
  }
}

units::degree_t AutoPositionAimCommand::GetTurretAngleToTarget(const frc::Pose2d curRobotPos,
                                                               frc::Translation2d target) {
  units::degree_t curRobotRotation = curRobotPos.Rotation().Degrees();
  units::degree_t angleToTarget = GetAngleToPoint(curRobotPos.Translation(), target);
  return argos_lib::swerve::ConstrainAngle(360_deg - (curRobotRotation - angleToTarget), 0_deg, 360_deg);
}
