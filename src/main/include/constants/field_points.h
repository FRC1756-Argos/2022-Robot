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
    static const frc::Translation2d A = frc::Translation2d(298_in, 11.5_in);
    static const frc::Translation2d B = frc::Translation2d(198.6_in, 74_in);
    static const frc::Translation2d C = frc::Translation2d(194_in, 242_in);
    static const frc::Translation2d D = frc::Translation2d(41.5_in, 44.4_in);
  }  // namespace balls
  namespace starting_positions {
    static const frc::Pose2d W = frc::Pose2d(298.5_in, 72.18_in, -90_deg);
    static const frc::Pose2d X = frc::Pose2d(272.81_in, 105.36_in, -147.5_deg);
    static const frc::Pose2d Y = frc::Pose2d(247.37_in, 175.67_in, 180_deg);
    static const frc::Pose2d Z = frc::Pose2d(252.5_in, 217.67_in, 135_deg);
  }  // namespace starting_positions
  namespace pickup_positions {
    static const frc::Pose2d W_A = frc::Pose2d(balls::A.X(),
                                               balls::A.Y() + (measure_up::chassis::length / 2) + 6_in,
                                               -90_deg);  ///< Drive so intake goes 1 bumper width past ball center
    static const frc::Pose2d W_A_B = frc::Pose2d(balls::B.X(),
                                                 balls::B.Y() + 1.5_ft,
                                                 -200_deg);          ///< Approach at 45 degree angle
    static const frc::Pose2d D = frc::Pose2d(balls::D.X(),           // + 12.75_in,
                                             balls::D.Y() + 3.2_ft,  // + 12.75_in,
                                             -135_deg);              ///< Approach at 45 degree angle
    static const frc::Pose2d Shoot_D = frc::Pose2d(balls::B.X(),
                                                   balls::B.Y(),
                                                   -135_deg);  ///< Approach at 45 degree angle
  }                                                            // namespace pickup_positions
}  // namespace field_points
