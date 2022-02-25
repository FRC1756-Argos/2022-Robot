/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/home_climber_arm_command.h"

HomeClimberArmCommand::HomeClimberArmCommand(ClimberSubsystem* subsystem)
    : m_pClimberSubsystem(subsystem), m_armMovingDebounce({0_ms, 500_ms}, true) {
  AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void HomeClimberArmCommand::Initialize() {
  m_pClimberSubsystem->MoveArm(-0.1);
}

// Called repeatedly when this Command is scheduled to run
void HomeClimberArmCommand::Execute() {
  if (m_pClimberSubsystem->IsManualOverride()) {
    Cancel();
  } else {
    m_pClimberSubsystem->MoveArm(-0.1);
  }
}

// Called once the command ends or is interrupted.
void HomeClimberArmCommand::End(bool interrupted) {
  if (!interrupted) {
    m_pClimberSubsystem->UpdateArmHome();
  }
  m_pClimberSubsystem->MoveArm(0.0);
}

// Returns true when the command should end.
bool HomeClimberArmCommand::IsFinished() {
  return !m_armMovingDebounce(m_pClimberSubsystem->IsArmMoving());
}
