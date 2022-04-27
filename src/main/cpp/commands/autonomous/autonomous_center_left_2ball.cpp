/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_center_left_2ball.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousCenterLeft2ball::AutonomousCenterLeft2ball(IntakeSubsystem* pIntake,
                                                     ShooterSubsystem* pShooter,
                                                     SwerveDriveSubsystem* pDrive)
    : m_pIntake{pIntake}
    , m_pShooter{pShooter}
    , m_pDrive{pDrive}
    , m_startDelay{600_ms}
    , m_driveToBallC{m_pDrive,
                     field_points::starting_positions::Z,
                     field_points::starting_positions::Z.Rotation().Degrees(),
                     field_points::pickup_positions::blue_alliance::Z_C,
                     field_points::pickup_positions::blue_alliance::Z_C.Rotation().Degrees(),
                     constraints::first_ball_path::linearConstraints,
                     constraints::first_ball_path::rotationalConstraints}
    , m_homeHoodCommand{m_pShooter}
    , m_intakeCommand{m_pIntake}
    , m_shootCommand{m_pIntake, 2, 1.2_s}
    , m_aimBallC{m_pShooter, field_points::pickup_positions::blue_alliance::Z_C}
    , m_visionAim{m_pShooter}
    , m_initOdometry{m_pDrive, field_points::starting_positions::Z}
    , m_allCommands{} {
  m_allCommands.AddCommands(
      frc2::ParallelCommandGroup{frc2::SequentialCommandGroup{m_homeHoodCommand, m_aimBallC},
                                 m_intakeCommand,
                                 frc2::SequentialCommandGroup{m_initOdometry, m_startDelay, m_driveToBallC}},
      m_visionAim,
      m_shootCommand,
      m_shootCommand);
}

// Called when the command is initially scheduled.
void AutonomousCenterLeft2ball::Initialize() {
  m_pShooter->SetCameraDriverMode(false);
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCenterLeft2ball::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCenterLeft2ball::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCenterLeft2ball::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousCenterLeft2ball::GetName() const {
  return "Center Left 2 Ball";
}

frc2::Command* AutonomousCenterLeft2ball::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
