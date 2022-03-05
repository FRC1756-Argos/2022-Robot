// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake_command.h"

Intake_command::Intake_command(IntakeSubsystem* subsystem) : m_pIntake(subsystem) {
  AddRequirements(m_pIntake);

  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Intake_command::Initialize() {m_pIntake->Intake();}

// Called repeatedly when this Command is scheduled to run
void Intake_command::Execute() {}

// Called once the command ends or is interrupted.
void Intake_command::End(bool interrupted) {m_pIntake->StopIntake();}

// Returns true when the command should end.
bool Intake_command::IsFinished() {
  bool ballPresent=m_pIntake->getBallPresentIntake ();
  if(!m_previousBallPresent&&ballPresent){
    m_previousBallPresent=ballPresent;
    return true;
  }
  m_previousBallPresent=ballPresent;
  return false;

}
