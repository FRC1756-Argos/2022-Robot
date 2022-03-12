/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>

#include "measure_up.h"

namespace field_points {
  static const frc::Translation2d hub = frc::Translation2d(324_in, 162_in);
  namespace balls {
    static const frc::Translation2d A = frc::Translation2d(297_in, 11.21_in);
    static const frc::Translation2d B = frc::Translation2d(76_in, 200_in);
    static const frc::Translation2d C = frc::Translation2d(194_in, 242_in);
    static const frc::Translation2d D = frc::Translation2d(42_in, 45_in);
  }  // namespace balls
  namespace starting_positions {
    static const frc::Pose2d W = frc::Pose2d(309.95_in, 72.18_in, -90_deg);
    static const frc::Pose2d X = frc::Pose2d(272.81_in, 105.36_in, -147.5_deg);
    static const frc::Pose2d Y = frc::Pose2d(247.37_in, 175.67_in, 180_deg);
    static const frc::Pose2d Z = frc::Pose2d(252.5_in, 217.67_in, 135_deg);
  }  // namespace starting_positions
  namespace pickup_positions {
    static const frc::Pose2d W_A =
        frc::Pose2d(starting_positions::W.X(),
                    starting_positions::W.Y() - (measure_up::chassis::length / 2) - measure_up::intakeExtension,
                    -90_deg);  ///< Drive so intake goes 1 bumper width past ball center
  }                            // namespace pickup_positions
}  // namespace field_points
