/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <chrono>

#include "argos_lib/general/debouncer.h"
#include "subsystems/shooter_subsystem.h"

class HomeHoodCommand : public frc2::CommandHelper<frc2::CommandBase, HomeHoodCommand> {
 public:
  explicit HomeHoodCommand(ShooterSubsystem* shooter);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_pShooter;
  argos_lib::Debouncer m_hoodMovingDebounce;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
};
