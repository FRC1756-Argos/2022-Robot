/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

namespace controlLoop {
  namespace drive {
    namespace rotate {
      constexpr double kP = 1.4;
      constexpr double kI = 0.0005;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 500.0;
      constexpr double allowableError = 0.0;
    }  // namespace rotate
  }    // namespace drive
  namespace shooter {
    namespace shooter {
      constexpr double kP = 0.02;
      constexpr double kI = 0.0;
      constexpr double kD = 0.0;
      constexpr double kF = 0.055;
      constexpr double iZone = 100.0;
      constexpr double allowableError = 0.0;
    }  // namespace shooter
    namespace hood {
      constexpr double kP = 0.75;
      constexpr double kI = 0.008;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 200.0;
      constexpr double allowableError = 0.0;
    }  // namespace hood
    namespace turret {
      constexpr double kP = 0.0;
      constexpr double kI = 0.0;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 200.0;
      constexpr double allowableError = 0.0;
    }  // namespace turret
  }    // namespace shooter
}  // namespace controlLoop
