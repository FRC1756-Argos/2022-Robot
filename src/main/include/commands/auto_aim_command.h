/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/shooter_subsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoAimCommand : public frc2::CommandHelper<frc2::CommandBase, AutoAimCommand> {
 public:
  AutoAimCommand(ShooterSubsystem* subsystem,
                 argos_lib::InterpolationMap<units::length::inch_t, 6, units::angle::degree_t>* hoodMap);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;
  argos_lib::InterpolationMap<units::length::inch_t, 6, units::angle::degree_t>* m_hoodMap;

  template <typename T>
  bool InThreshold(T value, T threshold);
};
