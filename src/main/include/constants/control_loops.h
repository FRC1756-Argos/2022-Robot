/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

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
      struct drive {
        constexpr static double kP = 0.11;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.05;
        constexpr static double iZone = 500.0;
        constexpr static double allowableError = 0.0;
      };  // namespace drive
      struct linear_follower {
        constexpr static double kP = 8.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
      };  // namespace linear_follower
      struct rotational_follower {
        constexpr static double kP = 8.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static auto angularVelocity = units::degrees_per_second_t{360};
        constexpr static auto angularAcceleration = units::degrees_per_second_squared_t{360};
      };  // namespace rotational_follower
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
        constexpr static double allowableError = 4;
      };  // namespace hood
      struct turret {
        constexpr static double kP = 4.25;
        constexpr static double kI = 0.05;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.0;
        constexpr static double iZone = 10.0;
        constexpr static double allowableError = 2;
      };  // namespace turret
    }     // namespace shooter
    namespace climber {
      namespace hook {
        constexpr static double kP = 0.2;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.05;
        constexpr static double iZone = 200.0;
        constexpr static double allowableError = 0;
      }  // namespace hook
      namespace arm {
        constexpr static double kP = 0.03;
        constexpr static double kI = 0.01;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.05;
        constexpr static double iZone = 200.0;
        constexpr static double allowableError = 0;
      }  // namespace arm
    }    // namespace climber

    namespace climber_loaded {
      namespace hook {
        constexpr static double kP = 0.7;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.15;
        constexpr static double iZone = 200.0;
        constexpr static double allowableError = 0;
      }  // namespace hook
      namespace arm {
        constexpr static double kP = 0.03;
        constexpr static double kI = 0.01;
        constexpr static double kD = 0.0;
        constexpr static double kF = 0.05;
        constexpr static double iZone = 200.0;
        constexpr static double allowableError = 0;
      }  // namespace arm
    }    // namespace climber_loaded

  }  // namespace comp_bot
  namespace practice_bot {
    namespace drive {
      using rotate = controlLoop::comp_bot::drive::rotate;
      using drive = controlLoop::comp_bot::drive::drive;
      using linear_follower = controlLoop::comp_bot::drive::linear_follower;
      using rotational_follower = controlLoop::comp_bot::drive::rotational_follower;
    }  // namespace drive
    namespace shooter {
      using shooter = controlLoop::comp_bot::shooter::shooter;
      using hood = controlLoop::comp_bot::shooter::hood;
      using turret = controlLoop::comp_bot::shooter::turret;
    }  // namespace shooter
  }    // namespace practice_bot
}  // namespace controlLoop
