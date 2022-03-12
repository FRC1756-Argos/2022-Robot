/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/length.h>

namespace fieldPoints {
  const frc::Translation2d hub = frc::Translation2d(324_in, 162_in);
  namespace balls {
    const frc::Translation2d A = frc::Translation2d(297_in, 11.21_in);
  }  // namespace balls
  namespace startingpositions {
    const frc::Translation2d W = frc::Translation2d(309.95_in, 72.18_in);
    const frc::Translation2d X = frc::Translation2d(272.81_in, 105.36_in);
    const frc::Translation2d Y = frc::Translation2d(247.37_in, 175.67_in);
    const frc::Translation2d Z = frc::Translation2d(252.5_in, 217.67_in);
  }  // namespace startingpositions
}  // namespace fieldPoints
