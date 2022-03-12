/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_right_2ball.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousRight2ball::AutonomousRight2ball(IntakeSubsystem* pIntake,
                                           ShooterSubsystem* pShooter,
                                           SwerveDriveSubsystem* pDrive)
    : m_pIntake{pIntake}
    , m_pShooter{pShooter}
    , m_pDrive{pDrive}
    , m_driveToBallA{m_pDrive,
                     frc::Pose2d{},
                     constraints::first_ball_path::linearConstraints,
                     constraints::first_ball_path::rotationalConstraints}  ///< @todo add position
    , m_homeHoodCommand{m_pShooter}
    , m_intakeCommand{m_pIntake}
    , m_shootCommand{m_pIntake}
    , m_aimBallA{m_pShooter, frc::Pose2d{}}
    , m_initOdometry{m_pDrive, field_points::starting_positions::W}
    , m_allCommands{} {
  m_allCommands.AddCommands(frc2::ParallelCommandGroup{frc2::SequentialCommandGroup{m_homeHoodCommand, m_aimBallA},
                                                       m_intakeCommand,
                                                       frc2::SequentialCommandGroup{m_initOdometry, m_driveToBallA}},
                            m_shootCommand);
}

// Called when the command is initially scheduled.
void AutonomousRight2ball::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousRight2ball::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousRight2ball::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousRight2ball::IsFinished() {
  return m_allCommands.IsFinished();
}
