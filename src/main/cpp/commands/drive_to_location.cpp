/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_to_location.h"

#include "utils/swerve_trapezoidal_profile.h"

DriveToLocation::DriveToLocation(SwerveDriveSubsystem* drive,
                                 const frc::Pose2d source,
                                 const frc::Pose2d destination,
                                 const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
                                 const frc::TrapezoidProfile<units::degrees>::Constraints rotationalConstraints)
    : m_pDrive(drive)
    , m_source(source)
    , m_destination(destination)
    , m_linearConstraints(linearConstraints)
    , m_rotationalConstraints(rotationalConstraints) {
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void DriveToLocation::Initialize() {
  m_pDrive->UpdateFollowerRotationalPIDConstraints(m_rotationalConstraints);
  m_pDrive->StartDrivingProfile(SwerveTrapezoidalProfileSegment{m_source, m_destination, m_linearConstraints});
}

// Called repeatedly when this Command is scheduled to run
void DriveToLocation::Execute() {
  // No need to do anything beyond initialize
}

// Called once the command ends or is interrupted.
void DriveToLocation::End(bool interrupted) {
  m_pDrive->StopDrive();
}

// Returns true when the command should end.
bool DriveToLocation::IsFinished() {
  return m_pDrive->ProfileIsComplete();
}
