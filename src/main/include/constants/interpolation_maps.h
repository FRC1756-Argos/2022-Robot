/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>

#include "argos_lib/general/interpolation.h"

namespace shooterRange {
  using argos_lib::InterpMapPoint;
  [[maybe_unused]] constexpr std::array shooterSpeed{InterpMapPoint{10.0_in, 2300.0_rpm},
                                                     InterpMapPoint{21.5_in, 2350.0_rpm},
                                                     InterpMapPoint{60.0_in, 2500.0_rpm},
                                                     InterpMapPoint{120.0_in, 2750.0_rpm},
                                                     InterpMapPoint{180.0_in, 3150.0_rpm},
                                                     InterpMapPoint{240.0_in, 3425.0_rpm},
                                                     InterpMapPoint{300.0_in, 3825.0_rpm},
                                                     InterpMapPoint{360.0_in, 4175.0_rpm}};
  [[maybe_unused]] constexpr std::array hoodAngle{InterpMapPoint{10.0_in, 5.0_deg},
                                                  InterpMapPoint{21.5_in, 7.0_deg},
                                                  InterpMapPoint{60.0_in, 13.0_deg},
                                                  InterpMapPoint{120.0_in, 20.0_deg},
                                                  InterpMapPoint{180.0_in, 29.0_deg},
                                                  InterpMapPoint{240.0_in, 38.0_deg},
                                                  InterpMapPoint{300.0_in, 41.0_deg},
                                                  InterpMapPoint{360.0_in, 42.0_deg}};
  [[maybe_unused]] constexpr std::array lateralSpeed{InterpMapPoint{10.0_in, 2.0_fps},
                                                     InterpMapPoint{21.5_in, 2.6_fps},
                                                     InterpMapPoint{60.0_in, 5.06_fps},
                                                     InterpMapPoint{120.0_in, 7.8_fps},
                                                     InterpMapPoint{180.0_in, 11.66_fps},
                                                     InterpMapPoint{240.0_in, 13.85_fps},
                                                     InterpMapPoint{300.0_in, 14.41_fps},
                                                     InterpMapPoint{360.0_in, 15.9_fps}};
}  // namespace shooterRange
namespace controllerMap {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array driveLongSpeed{InterpMapPoint{-1.0, -1.0},
                                                       InterpMapPoint{-0.75, -0.4},
                                                       InterpMapPoint{-0.15, 0.0},
                                                       InterpMapPoint{0.15, 0.0},
                                                       InterpMapPoint{0.75, 0.4},
                                                       InterpMapPoint{1.0, 1.0}};
  [[maybe_unused]] constexpr std::array driveLatSpeed{InterpMapPoint{-1.0, -1.0},
                                                      InterpMapPoint{-0.75, -0.4},
                                                      InterpMapPoint{-0.15, 0.0},
                                                      InterpMapPoint{0.15, 0.0},
                                                      InterpMapPoint{0.75, 0.4},
                                                      InterpMapPoint{1.0, 1.0}};
  [[maybe_unused]] constexpr std::array driveRotSpeed{
      InterpMapPoint{-1.0, -1.0}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 1.0}};

  [[maybe_unused]] constexpr std::array hookSpeed{
      InterpMapPoint{-1.0, -0.8}, InterpMapPoint{-0.2, 0.0}, InterpMapPoint{0.2, 0.0}, InterpMapPoint{1.0, 0.8}};
  [[maybe_unused]] constexpr std::array armSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.2, 0.0}, InterpMapPoint{0.2, 0.0}, InterpMapPoint{1.0, 0.6}};

  [[maybe_unused]] constexpr std::array turretSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array hoodSpeed{
      InterpMapPoint{-1.0, -0.4}, InterpMapPoint{-0.25, 0.0}, InterpMapPoint{0.25, 0.0}, InterpMapPoint{1.0, 0.4}};
}  // namespace controllerMap
