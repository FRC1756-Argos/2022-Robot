/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <chrono>

#include "argos_lib/general/debouncer.h"
#include "subsystems/climber_subsystem.h"

class HomeClimberArmCommand : public frc2::CommandHelper<frc2::CommandBase, HomeClimberArmCommand> {
 public:
  explicit HomeClimberArmCommand(ClimberSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ClimberSubsystem* m_pClimberSubsystem;
  argos_lib::Debouncer m_armMovingDebounce;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
};
