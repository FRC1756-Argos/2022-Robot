/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>

namespace measure_up {
  namespace chassis {
    constexpr units::inch_t width{28.0};
    constexpr units::inch_t length{31.0};
  }  // namespace chassis
  namespace swerve_offsets {
    constexpr auto frontLeftLOffset = 4.0_in;
    constexpr auto frontLeftWOffset = 4.0_in;
    constexpr auto frontRightLOffset = 4.0_in;
    constexpr auto frontRightWOffset = 4.0_in;
    constexpr auto backRightWOffset = 4.0_in;
    constexpr auto backRightLOffset = 4.0_in;
    constexpr auto backLeftWOffset = 4.0_in;
    constexpr auto backLeftLOffset = 4.0_in;
  }  // namespace swerve_offsets
  namespace hood {
    constexpr auto homeAngle = 1_deg * -1;
  }  // namespace hood
  namespace turret {
    constexpr auto homeAngle = 180_deg;
    constexpr auto minAngle = 10_deg * -1;
    constexpr auto maxAngle = 370_deg;
  }  // namespace turret
  namespace camera {
    constexpr auto cameraHeight = 28.5_in;
    constexpr auto upperHubHeight = 104_in;
    constexpr auto cameraMountAngle = 39_deg;
    constexpr auto toRotationCenter = 9_in;
  }  // namespace camera
}  // namespace measure_up
