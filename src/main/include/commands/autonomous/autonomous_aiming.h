/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "argos_lib/general/debouncer.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "utils/sensor_conversions.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutonomousAiming : public frc2::CommandHelper<frc2::CommandBase, AutonomousAiming> {
 public:
  explicit AutonomousAiming(ShooterSubsystem* shootSys, SwerveDriveSubsystem* driveSys);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;
  SwerveDriveSubsystem* m_swerveDrive;
  argos_lib::Debouncer m_threshDebounce;

  units::degree_t GetOffsetAngle(frc::Translation2d curPos, frc::Translation2d target);

  units::degree_t GetAngleToPoint(frc::Translation2d curPos, frc::Translation2d target);

  units::degree_t GetTurretAngleToTarget(frc::Pose2d curRobotPos, frc::Translation2d target);
};
