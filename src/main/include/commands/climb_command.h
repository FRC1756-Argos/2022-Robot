/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <queue>
#include <vector>

#include "subsystems/climber_subsystem.h"

/**
 * @brief Works with the climber subsystem to execute a series of setpoints, automating the climb sequence
 *
 */
class ClimbCommand : public frc2::CommandHelper<frc2::CommandBase, ClimbCommand> {
 public:
  ClimbCommand(ClimberSubsystem* subsystem, std::vector<ClimberPoint> points);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ClimberSubsystem* m_pClimberSubsystem;
  std::vector<ClimberPoint> m_initPoints;
  std::queue<ClimberPoint> m_climbPoints;
};
