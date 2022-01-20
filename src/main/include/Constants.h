/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <units/length.h>

#include "argos_lib/general/interpolation.h"

namespace address {
  namespace motor {
    constexpr const char frontLeftDrive = NULL;
    constexpr const char frontLeftTurn = NULL;
    constexpr const char frontRightDrive = NULL;
    constexpr const char frontRightTurn = NULL;
    constexpr const char backRightDrive = NULL;
    constexpr const char backRightTurn = NULL;
    constexpr const char backLeftDrive = NULL;
    constexpr const char backLeftTurn = NULL;
  }  // namespace motor
  namespace encoders {
    constexpr const char frontLeftEncoder = NULL;
    constexpr const char frontRightEncoder = NULL;
    constexpr const char backRightEncoder = NULL;
    constexpr const char backLeftEncoder = NULL;
  }  // namespace encoders
  namespace controllers {
    constexpr const char driver = NULL;
    constexpr const char secondary = NULL;
  }  // namespace controllers

}  // namespace address

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
}  // namespace measure_up

namespace indexes {
  namespace swerveModules {
    constexpr char frontLeftIndex = 0;
    constexpr char frontRightIndex = 1;
    constexpr char backRightIndex = 2;
    constexpr char backLeftIndex = 3;
  }  // namespace swerveModules
}  // namespace indexes

namespace controllerMap {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array driveLongSpeed{InterpMapPoint{-1.0, 0.6},
                                                       InterpMapPoint{-0.75, 0.4},
                                                       InterpMapPoint{-0.15, 0.0},
                                                       InterpMapPoint{0.15, 0.0},
                                                       InterpMapPoint{0.75, -0.4},
                                                       InterpMapPoint{1.0, -0.6}};
  [[maybe_unused]] constexpr std::array driveLatSpeed{InterpMapPoint{-1.0, -0.6},
                                                      InterpMapPoint{-0.75, -0.4},
                                                      InterpMapPoint{-0.15, 0.0},
                                                      InterpMapPoint{0.15, 0.0},
                                                      InterpMapPoint{0.75, 0.4},
                                                      InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array driveRotSpeed{
      InterpMapPoint{-1.0, -1.0}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 1.0}};
}  // namespace controllerMap
