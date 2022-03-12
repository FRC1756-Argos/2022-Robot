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
    namespace comp_bot {
      constexpr auto homeAngle = 0.6_deg * -1;
    }  // namespace comp_bot
    namespace practice_bot {
      constexpr auto homeAngle = 1.2_deg * -1;
    }  // namespace practice_bot
  }    // namespace hood
  namespace turret {
    constexpr auto homeAngle = 180_deg;
    constexpr auto minAngle = 45_deg;
    constexpr auto maxAngle = 360_deg;
  }  // namespace turret
  namespace camera {
    constexpr auto cameraHeight = 28.5_in;
    constexpr auto upperHubHeight = 104_in;
    constexpr auto cameraMountAngle = 27_deg;               // actual current camera mount angle is 27.8 degrees
    constexpr auto cameraMountAnglePracticeBot = 28.3_deg;  // actual currect camera mount angle is 29.1
    constexpr auto toRotationCenter = 9_in;
  }  // namespace camera
  namespace climber_arm {
    constexpr auto homeExtension = 21.5_in;  ///< Length between mount points
  }                                          // namespace climber_arm
  namespace climber_hook {
    constexpr auto homeExtension = 35.25_in;  ///< Length from arm pivot to inner edge of hook slider
    constexpr auto maxExtension = 35.00_in;
    constexpr auto minExtension = 1_in;
  }  // namespace climber_hook
  namespace closepositions {
    constexpr auto fixedLongDist = 21.5_in;   //< Front and back distance from shooter to frame perimeter
    constexpr auto fixedShortDist = 18.5_in;  //< Left and right distance from shooter to frame perimeter
    constexpr auto fixedFrontPos = 360_deg;
    constexpr auto fixedLeftPos = 90_deg;
    constexpr auto fixedBackPos = 180_deg;
    constexpr auto fixedRightPos = 270_deg;
  }  // namespace closepositions
}  // namespace measure_up
