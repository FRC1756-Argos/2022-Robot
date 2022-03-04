/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

namespace controlLoop {
  namespace comp_bot {
    namespace drive {
      struct rotate {
        constexpr static double kP = 1.4;
        constexpr static double kI = 0.0005;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.0;
        constexpr static double iZone = 500.0;
        constexpr static double allowableError = 0.0;
      };  // namespace rotate
    }     // namespace drive
    namespace shooter {
      struct shooter {
        constexpr static double kP = 0.02;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.055;
        constexpr static double iZone = 100.0;
        constexpr static double allowableError = 0.0;
      };  // namespace shooter
      struct hood {
        constexpr static double kP = 0.75;
        constexpr static double kI = 0.008;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.0;
        constexpr static double iZone = 200.0;
        constexpr static double allowableError = 0.0;
      };  // namespace hood
      struct turret {
        constexpr static double kP = 4.25;
        constexpr static double kI = 0.05;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.0;
        constexpr static double iZone = 10.0;
        constexpr static double allowableError = 0.0;
      };  // namespace turret
    }     // namespace shooter
  }       // namespace comp_bot
  namespace practice_bot {
    namespace drive {
      using rotate = controlLoop::comp_bot::drive::rotate;
    }  // namespace drive
    namespace shooter {
      using shooter = controlLoop::comp_bot::shooter::shooter;
      using hood = controlLoop::comp_bot::shooter::hood;
      using turret = controlLoop::comp_bot::shooter::turret;
    }  // namespace shooter
  }    // namespace practice_bot
}  // namespace controlLoop
