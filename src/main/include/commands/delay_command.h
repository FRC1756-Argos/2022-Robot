/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <units/time.h>

#include <chrono>

class DelayCommand : public frc2::CommandHelper<frc2::CommandBase, DelayCommand> {
 public:
  explicit DelayCommand(units::millisecond_t delayTime);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;  ///< When the command began
  units::millisecond_t m_timeout;                                  ///< 0 indicates no timeout
};
