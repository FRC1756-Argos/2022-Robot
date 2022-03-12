/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "utils/sensor_conversions.h"

namespace constraints {
  namespace first_ball_path {
    static const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints{
        12_fps, units::feet_per_second_squared_t{12}};
    static const frc::TrapezoidProfile<units::degrees>::Constraints rotationalConstraints{
        units::degrees_per_second_t{360}, units::degrees_per_second_squared_t{360}};
  }  // namespace first_ball_path
}  // namespace constraints
