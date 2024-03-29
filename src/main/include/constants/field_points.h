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
    static const frc::Translation2d B = frc::Translation2d(198.4_in, 74_in);
    static const frc::Translation2d C = frc::Translation2d(209.3_in, 244.4_in);
    static const frc::Translation2d D = frc::Translation2d(42_in, 45_in);
    static const frc::Translation2d F = frc::Translation2d(184.4_in, 124.6_in);
  }  // namespace balls
  namespace starting_positions {
    static const frc::Pose2d W = frc::Pose2d(298.5_in, 72.18_in, -90_deg);
    static const frc::Pose2d X = frc::Pose2d(261.2_in, 105.6_in, -147.5_deg);
    static const frc::Pose2d Y = frc::Pose2d(247.37_in, 150.6_in, 180_deg);
    static const frc::Pose2d Z = frc::Pose2d(252.3_in, 204.2_in, 135_deg);
  }  // namespace starting_positions
  namespace pickup_positions {
    namespace blue_alliance {
      static const frc::Pose2d W_A = frc::Pose2d(balls::A.X(),
                                                 balls::A.Y() + (measure_up::chassis::length / 2) + 5_in,
                                                 -90_deg);  ///< Drive so intake goes 1 bumper width past ball center

      static const frc::Pose2d W_A_B = frc::Pose2d(balls::B.X(), balls::B.Y() + 1.5_ft, -200_deg);

      static const frc::Pose2d D = frc::Pose2d(balls::D.X() + 10_in,
                                               balls::D.Y() + 15_in,
                                               -135_deg);  ///< Approach at 45 degree angle

      static const frc::Pose2d HumanPlayer = frc::Pose2d(pickup_positions::blue_alliance::D.X() + 6_in,
                                                         pickup_positions::blue_alliance::D.Y() + 6_in,
                                                         pickup_positions::blue_alliance::D.Rotation());

      static const frc::Pose2d Shoot_D = frc::Pose2d(balls::B.X(),
                                                     balls::B.Y(),
                                                     -45_deg);  ///< Approach at 45 degree angle

      static const frc::Pose2d Shoot_F = frc::Pose2d(balls::F.X() + 12_in,
                                                     balls::F.Y(),
                                                     -270_deg);  ///< Approach at 45 degree angle

      static const frc::Pose2d Virtual_Shoot_F = frc::Pose2d(454.1_in, 77.1_in, Shoot_F.Rotation());

      static const frc::Pose2d X_B = frc::Pose2d(balls::B.X() + 6_in, balls::B.Y() - 12_in, -147.5_deg);

      static const frc::Pose2d Z_C = frc::Pose2d(balls::C.X(), balls::C.Y(), 135_deg);

      static const frc::Pose2d Y_Y = frc::Pose2d(starting_positions::Y.X() - 48_in, starting_positions::Y.Y(), 180_deg);

      static const frc::Pose2d Defense_Virt = frc::Pose2d(500_in, 162_in, 225_deg);
      static const frc::Pose2d Defense_Right = frc::Pose2d(174_in, 126_in, 270_deg);
      static const frc::Pose2d Defense_Left = frc::Pose2d(234_in + 6_in, 228_in + 60_in, 405_deg);
    }  // namespace blue_alliance
    namespace red_alliance {
      static const frc::Pose2d W_A = frc::Pose2d(balls::A.X(),
                                                 balls::A.Y() + (measure_up::chassis::length / 2) + 5_in,
                                                 -90_deg);  ///< Drive so intake goes 1 bumper width past ball center

      static const frc::Pose2d W_A_B = frc::Pose2d(balls::B.X(), balls::B.Y() + 1.5_ft, -200_deg);

      static const frc::Pose2d D = frc::Pose2d(balls::D.X(),
                                               balls::D.Y() + 25_in,
                                               -135_deg);  ///< Approach at 45 degree angle

      static const frc::Pose2d HumanPlayer = frc::Pose2d(pickup_positions::blue_alliance::D.X() + 6_in,
                                                         pickup_positions::blue_alliance::D.Y() + 6_in,
                                                         pickup_positions::blue_alliance::D.Rotation());

      static const frc::Pose2d Shoot_D = frc::Pose2d(balls::B.X(),
                                                     balls::B.Y(),
                                                     -45_deg);  ///< Approach at 45 degree angle

      static const frc::Pose2d Shoot_F = frc::Pose2d(balls::F.X() + 12_in,
                                                     balls::F.Y(),
                                                     -270_deg);  ///< Approach at 45 degree angle

      static const frc::Pose2d Virtual_Shoot_F = frc::Pose2d(454.1_in, 77.1_in, Shoot_F.Rotation());

      static const frc::Pose2d X_B = frc::Pose2d(balls::B.X() + 6_in, balls::B.Y() - 12_in, -147.5_deg);

      static const frc::Pose2d Z_C = frc::Pose2d(balls::C.X(), balls::C.Y(), 135_deg);

      static const frc::Pose2d Y_Y = frc::Pose2d(starting_positions::Y.X() - 48_in, starting_positions::Y.Y(), 180_deg);

      static const frc::Pose2d Defense_Virt = frc::Pose2d(500_in, 162_in, 225_deg);
      static const frc::Pose2d Defense_Right = frc::Pose2d(174_in, 126_in, 270_deg);
      static const frc::Pose2d Defense_Left = frc::Pose2d(234_in + 6_in, 228_in + 60_in, 405_deg);
    }  // namespace red_alliance

  }  // namespace pickup_positions

}  // namespace field_points
