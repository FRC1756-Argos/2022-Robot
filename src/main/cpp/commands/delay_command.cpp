/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/delay_command.h"

DelayCommand::DelayCommand(units::millisecond_t delayTime) : m_timeout{delayTime} {}

// Called when the command is initially scheduled.
void DelayCommand::Initialize() {
  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void DelayCommand::Execute() {}

// Called once the command ends or is interrupted.
void DelayCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool DelayCommand::IsFinished() {
  return units::millisecond_t{std::chrono::steady_clock::now() - m_startTime} > m_timeout;
}
