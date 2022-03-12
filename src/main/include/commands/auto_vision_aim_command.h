/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "argos_lib/general/debouncer.h"
#include "subsystems/shooter_subsystem.h"

class AutoVisionAimCommand : public frc2::CommandHelper<frc2::CommandBase, AutoVisionAimCommand> {
 public:
  explicit AutoVisionAimCommand(ShooterSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;
  argos_lib::Debouncer m_threshDebounce;

  template <typename T>
  bool InThreshold(T value, T threshold);

  bool InAcceptableRanges(AimValues targets, AimValues real);
};
