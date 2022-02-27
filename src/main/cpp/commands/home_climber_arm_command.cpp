/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_climber_arm_command.h"

using namespace std::chrono_literals;

HomeClimberArmCommand::HomeClimberArmCommand(ClimberSubsystem* subsystem)
    : m_pClimberSubsystem(subsystem)
    , m_armMovingDebounce({0_ms, 500_ms}, true)
    , m_startTime(std::chrono::steady_clock::now()) {
  if (subsystem != nullptr) {
    AddRequirements(subsystem);
  }
}

// Called when the command is initially scheduled.
void HomeClimberArmCommand::Initialize() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }
  m_pClimberSubsystem->MoveArm(-0.1);
}

// Called repeatedly when this Command is scheduled to run
void HomeClimberArmCommand::Execute() {
  if (m_pClimberSubsystem == nullptr) {
    Cancel();
    return;
  }
  if (m_pClimberSubsystem->IsManualOverride() || std::chrono::steady_clock::now() - m_startTime > 2.0s) {
    Cancel();
  } else {
    m_pClimberSubsystem->MoveArm(-0.1);
  }
}

// Called once the command ends or is interrupted.
void HomeClimberArmCommand::End(bool interrupted) {
  if (m_pClimberSubsystem == nullptr) {
    return;
  }
  if (!interrupted) {
    m_pClimberSubsystem->UpdateArmHome();
  }
  m_pClimberSubsystem->MoveArm(0.0);
}

// Returns true when the command should end.
bool HomeClimberArmCommand::IsFinished() {
  if (m_pClimberSubsystem == nullptr) {
    return true;
  }
  return !m_armMovingDebounce(m_pClimberSubsystem->IsArmMoving());
}
