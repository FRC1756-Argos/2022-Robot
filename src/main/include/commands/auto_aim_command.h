/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "argos_lib/general/debouncer.h"
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
  explicit AutoAimCommand(ShooterSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  struct aimValues {
    units::degree_t turretTarget;
    units::degree_t hoodTarget;
    units::angular_velocity::revolutions_per_minute_t shooterTarget;
  };

  ShooterSubsystem* m_shooter;
  argos_lib::Debouncer m_threshDebounce;

  template <typename T>
  bool InThreshold(T value, T threshold);

  bool InAcceptableRanges(AutoAimCommand::aimValues targets, AutoAimCommand::aimValues real);
};
