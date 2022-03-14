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

class AutoPositionAimCommand : public frc2::CommandHelper<frc2::CommandBase, AutoPositionAimCommand> {
 public:
  explicit AutoPositionAimCommand(ShooterSubsystem* shootSys, frc::Pose2d shootingPosition);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_pShooter;
  frc::Pose2d m_shootingPosition;
  argos_lib::Debouncer m_threshDebounce;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;  ///< When the command began

  units::degree_t GetOffsetAngle(frc::Translation2d curPos, frc::Translation2d target);

  units::degree_t GetAngleToPoint(frc::Translation2d curPos, frc::Translation2d target);

  units::degree_t GetTurretAngleToTarget(frc::Pose2d curRobotPos, frc::Translation2d target);
};
