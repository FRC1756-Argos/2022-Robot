/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_center_left_2ball_defense.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousCenterLeft2ballDefense::AutonomousCenterLeft2ballDefense(IntakeSubsystem* pIntake,
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
    , m_driveDefenseRight{m_pDrive,
                          field_points::pickup_positions::blue_alliance::Z_C,
                          field_points::pickup_positions::blue_alliance::Z_C.Rotation().Degrees(),
                          field_points::pickup_positions::blue_alliance::Defense_Right,
                          field_points::pickup_positions::blue_alliance::Defense_Right.Rotation().Degrees(),
                          constraints::first_ball_path::linearConstraints,
                          constraints::first_ball_path::rotationalConstraints}
    , m_driveDefenseLeft{m_pDrive,
                         field_points::pickup_positions::blue_alliance::Defense_Right,
                         field_points::pickup_positions::blue_alliance::Defense_Right.Rotation().Degrees(),
                         field_points::pickup_positions::blue_alliance::Defense_Left,
                         field_points::pickup_positions::blue_alliance::Defense_Left.Rotation().Degrees(),
                         constraints::first_ball_path::linearConstraints,
                         constraints::first_ball_path::rotationalConstraints}
    , m_homeHoodCommand{m_pShooter}
    , m_intakeCommand{m_pIntake}
    , m_shootCommand{m_pIntake, 2, 2.5_s}
    , m_aimBallC{m_pShooter, field_points::pickup_positions::blue_alliance::Z_C}
    , m_aimDefenseBottom{m_pShooter, field_points::pickup_positions::blue_alliance::Defense_Virt}
    , m_visionAim{m_pShooter}
    , m_initOdometry{m_pDrive, field_points::starting_positions::Z}
    , m_allCommands{} {
  m_allCommands.AddCommands(
      frc2::ParallelCommandGroup{frc2::SequentialCommandGroup{m_homeHoodCommand, m_aimBallC},
                                 m_intakeCommand,
                                 frc2::SequentialCommandGroup{m_initOdometry, m_startDelay, m_driveToBallC}},
      m_visionAim,
      m_shootCommand,
      m_startDelay,
      frc2::ParallelCommandGroup{m_aimDefenseBottom,
                                 frc2::SequentialCommandGroup{m_driveDefenseRight, m_driveDefenseLeft}},
      m_shootCommand);
}

// Called when the command is initially scheduled.
void AutonomousCenterLeft2ballDefense::Initialize() {
  m_pShooter->SetCameraDriverMode(false);
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCenterLeft2ballDefense::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCenterLeft2ballDefense::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCenterLeft2ballDefense::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousCenterLeft2ballDefense::GetName() const {
  return "Center Left 2 Ball + Defense Hangar Shots";
}

frc2::Command* AutonomousCenterLeft2ballDefense::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
