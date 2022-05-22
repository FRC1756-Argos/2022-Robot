/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/time.h>

#include <chrono>

#include "subsystems/intake_subsystem.h"

class ShootCommand : public frc2::CommandHelper<frc2::CommandBase, ShootCommand> {
 public:
  explicit ShootCommand(IntakeSubsystem* subsystem);
  ShootCommand(IntakeSubsystem* subsystem, unsigned int numCargo, units::millisecond_t timeout);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem* m_pIntake;
  bool m_previousBallPresent;
  unsigned int m_totalCargo;  ///< Total cargo we expect to shoot.  0 indicates shoot until timeout
  unsigned int m_cargoShot;   ///< Cargo shot so far

  std::chrono::time_point<std::chrono::steady_clock> m_startTime;  ///< When the command began
  units::millisecond_t m_timeout;                                  ///< 0 indicates no timeout
};
