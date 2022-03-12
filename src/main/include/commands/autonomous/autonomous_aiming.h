/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "utils/sensor_conversions.h"
#include "argos_lib/general/debouncer.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutonomousAiming : public frc2::CommandHelper<frc2::CommandBase, AutonomousAiming> {
 public:
  AutonomousAiming(ShooterSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;
  SwerveDriveSubsystem* m_swerveDrive;
  argos_lib::Debouncer m_threshDebounce;
};
