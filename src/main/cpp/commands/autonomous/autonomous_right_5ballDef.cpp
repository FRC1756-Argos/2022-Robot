/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_right_5ballDef.h"

#include <frc2/command/ParallelCommandGroup.h>

#include "constants/constraints.h"
#include "constants/field_points.h"

AutonomousRight5ballDef::AutonomousRight5ballDef(IntakeSubsystem* pIntake,
                                                 ShooterSubsystem* pShooter,
                                                 SwerveDriveSubsystem* pDrive)
    : m_pIntake{pIntake}
    , m_pShooter{pShooter}
    , m_pDrive{pDrive}
    , m_5ball{pIntake, pShooter, pDrive}
    , m_shootDelay{300_ms}
    , m_driveToBallF{m_pDrive,
                     field_points::pickup_positions::blue_alliance::Shoot_D,
                     field_points::pickup_positions::blue_alliance::Shoot_D.Rotation().Degrees(),
                     field_points::pickup_positions::blue_alliance::Shoot_F,
                     field_points::pickup_positions::blue_alliance::Shoot_F.Rotation().Degrees(),
                     constraints::BD_path::linearConstraints,
                     constraints::BD_path::rotationalConstraints}
    , m_shootOneBall{m_pIntake, 1, 1.2_s}
    , m_aimBallF{m_pShooter, field_points::pickup_positions::blue_alliance::Virtual_Shoot_F}
    , m_allCommands{} {
  m_allCommands.AddCommands(AutonomousRight5ballGap{m_pIntake, m_pShooter, m_pDrive},
                            m_shootDelay,
                            frc2::ParallelCommandGroup{m_aimBallF, m_driveToBallF},
                            m_shootOneBall);
}

// Called when the command is initially scheduled.
void AutonomousRight5ballDef::Initialize() {
  m_pShooter->SetCameraDriverMode(false);
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousRight5ballDef::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousRight5ballDef::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousRight5ballDef::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousRight5ballDef::GetName() const {
  return "Right 5 Ball w/ Defense";
}

frc2::Command* AutonomousRight5ballDef::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
