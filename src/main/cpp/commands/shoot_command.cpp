/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/shoot_command.h"

ShootCommand::ShootCommand(IntakeSubsystem* subsystem) : ShootCommand(subsystem, 1, 0_ms) {}

ShootCommand::ShootCommand(IntakeSubsystem* subsystem, uint numCargo, units::millisecond_t timeout)
    : m_pIntake{subsystem}, m_totalCargo{numCargo}, m_cargoShot{0}, m_timeout{timeout} {
  AddRequirements(m_pIntake);
}

// Called when the command is initially scheduled.
void ShootCommand::Initialize() {
  m_previousBallPresent = false;
  m_cargoShot = 0;
  m_startTime = std::chrono::steady_clock::now();
  m_pIntake->Shoot();
}

// Called repeatedly when this Command is scheduled to run
void ShootCommand::Execute() {
  bool ballPresent = m_pIntake->getBallPresentShooter();
  if (m_previousBallPresent && !ballPresent) {
    m_previousBallPresent = ballPresent;
    ++m_cargoShot;
  }
  m_previousBallPresent = ballPresent;
}

// Called once the command ends or is interrupted.
void ShootCommand::End(bool interrupted) {
  m_pIntake->StopShoot();
}

// Returns true when the command should end.
bool ShootCommand::IsFinished() {
  return (m_totalCargo > 0 && m_cargoShot >= m_totalCargo) ||
         (m_timeout > 0_ms && units::millisecond_t{std::chrono::steady_clock::now() - m_startTime} > m_timeout);
}
