/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_center_1ball.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousCenter1ball::AutonomousCenter1ball(IntakeSubsystem* pIntake,
                                             ShooterSubsystem* pShooter,
                                             SwerveDriveSubsystem* pDrive)
    : m_pIntake{pIntake}
    , m_pShooter{pShooter}
    , m_pDrive{pDrive}
    , m_startDelay{250_ms}
    , m_driveToLocation{m_pDrive,
                        field_points::starting_positions::Y,
                        field_points::starting_positions::Y.Rotation().Degrees(),
                        field_points::pickup_positions::blue_alliance::Y_Y,
                        field_points::pickup_positions::blue_alliance::Y_Y.Rotation().Degrees(),
                        constraints::first_ball_path::linearConstraints,
                        constraints::first_ball_path::rotationalConstraints}
    , m_homeHoodCommand{m_pShooter}
    , m_intakeCommand{m_pIntake}
    , m_shootCommand{m_pIntake, 2, 1.2_s}
    , m_aimBall{m_pShooter, field_points::pickup_positions::blue_alliance::Y_Y}
    , m_visionAim{m_pShooter}
    , m_initOdometry{m_pDrive, field_points::starting_positions::Y}
    , m_allCommands{} {
  m_allCommands.AddCommands(
      frc2::ParallelCommandGroup{frc2::SequentialCommandGroup{m_homeHoodCommand, m_aimBall},
                                 m_intakeCommand,
                                 frc2::SequentialCommandGroup{m_initOdometry, m_startDelay, m_driveToLocation}},
      m_visionAim,
      m_shootCommand,
      m_shootCommand);
}

// Called when the command is initially scheduled.
void AutonomousCenter1ball::Initialize() {
  m_pShooter->SetCameraDriverMode(false);
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCenter1ball::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCenter1ball::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCenter1ball::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousCenter1ball::GetName() const {
  return "Center 1 Ball";
}

frc2::Command* AutonomousCenter1ball::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
