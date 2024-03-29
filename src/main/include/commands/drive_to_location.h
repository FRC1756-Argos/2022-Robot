/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <memory>

#include "subsystems/swerve_drive_subsystem.h"

class DriveToLocation : public frc2::CommandHelper<frc2::CommandBase, DriveToLocation> {
 public:
  DriveToLocation(SwerveDriveSubsystem* drive,
                  const frc::Pose2d source,
                  const units::degree_t sourceAngle,
                  const frc::Pose2d destination,
                  const units::degree_t destAngle,
                  const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
                  const frc::TrapezoidProfile<units::degrees>::Constraints rotationalConstraints);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pDrive;
  const frc::Pose2d m_source;
  const units::degree_t m_sourceAngle;
  const frc::Pose2d m_destination;
  const units::degree_t m_destAngle;
  const frc::TrapezoidProfile<units::inches>::Constraints m_linearConstraints;
  const frc::TrapezoidProfile<units::degrees>::Constraints m_rotationalConstraints;
};
