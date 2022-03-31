/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_right_5ball_gap.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousRight5ballGap::AutonomousRight5ballGap(IntakeSubsystem* pIntake,
                                                 ShooterSubsystem* pShooter,
                                                 SwerveDriveSubsystem* pDrive)
    : m_pIntake{pIntake}
    , m_pShooter{pShooter}
    , m_pDrive{pDrive}  // , m_2ball{pIntake, pShooter, pDrive}
    , m_humanPlayerDelay{1.5_s}
    , m_shootDelay{300_ms}
    , m_driveToBallB{m_pDrive,
                     field_points::pickup_positions::W_A,
                     field_points::pickup_positions::W_A.Rotation().Degrees(),
                     field_points::pickup_positions::W_A_B,
                     field_points::pickup_positions::W_A_B.Rotation().Degrees(),
                     constraints::AB_path::linearConstraints,
                     constraints::AB_path::rotationalConstraints}
    , m_driveToBallD{m_pDrive,
                     field_points::pickup_positions::W_A_B,
                     field_points::pickup_positions::W_A_B.Rotation().Degrees(),
                     field_points::pickup_positions::D,
                     field_points::pickup_positions::D.Rotation().Degrees(),
                     constraints::BD_path::linearConstraints,
                     constraints::BD_path::rotationalConstraints}
    , m_driveBackFromTerminal{m_pDrive,
                              field_points::pickup_positions::D,
                              field_points::pickup_positions::D.Rotation().Degrees(),
                              field_points::pickup_positions::HumanPlayer,
                              field_points::pickup_positions::HumanPlayer.Rotation().Degrees(),
                              constraints::terminal_gap_path::linearConstraints,
                              constraints::terminal_gap_path::rotationalConstraints}
    , m_driveToFinalShootPosition{m_pDrive,
                                  field_points::pickup_positions::HumanPlayer,
                                  field_points::pickup_positions::HumanPlayer.Rotation().Degrees(),
                                  field_points::pickup_positions::Shoot_D,
                                  field_points::pickup_positions::Shoot_D.Rotation().Degrees(),
                                  constraints::D_5_final::linearConstraints,
                                  constraints::D_5_final::rotationalConstraints}
    , m_shootOneBall{m_pIntake, 1, 1.2_s}
    , m_shootTwoBall{m_pIntake, 2, 1.5_s}
    , m_aimBallB{m_pShooter, field_points::pickup_positions::W_A_B}
    , m_aimBallD{m_pShooter, field_points::pickup_positions::Shoot_D}
    , m_visionAim{m_pShooter}
    , m_allCommands{} {
  m_allCommands.AddCommands(
      AutonomousRight2ball{m_pIntake, m_pShooter, m_pDrive},
      m_shootDelay,
      frc2::ParallelCommandGroup{m_aimBallB, m_driveToBallB},
      m_visionAim,
      m_shootOneBall,
      m_shootDelay,
      frc2::ParallelCommandGroup{
          m_aimBallD,
          frc2::SequentialCommandGroup{
              m_driveToBallD, m_driveBackFromTerminal, m_humanPlayerDelay, m_driveToFinalShootPosition}},
      m_visionAim,
      m_shootTwoBall,
      m_shootTwoBall);
}

// Called when the command is initially scheduled.
void AutonomousRight5ballGap::Initialize() {
  m_pShooter->SetCameraDriverMode(false);
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousRight5ballGap::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousRight5ballGap::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousRight5ballGap::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousRight5ballGap::GetName() const {
  return "Right 5 Ball w/ Gap";
}

frc2::Command* AutonomousRight5ballGap::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
