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
  [[maybe_unused]] constexpr std::array shooterSpeed{
      InterpMapPoint{10.0_in, 2300.0_rpm},
      InterpMapPoint{21.5_in, 2550.0_rpm},  // Tested 2022-02-27 reverse short shot
      InterpMapPoint{60.0_in, 2600.0_rpm},
      InterpMapPoint{120.0_in, 3000.0_rpm},  // Tested 2022-02-27, 50% accuracy
      InterpMapPoint{180.0_in, 3000.0_rpm},
      InterpMapPoint{240.0_in, 3500.0_rpm}};
  [[maybe_unused]] constexpr std::array hoodAngle{
      InterpMapPoint{10.0_in, 10.0_deg},
      InterpMapPoint{21.5_in, 13.5_deg},  // Tested 2022-02-27 reverse short shot
      InterpMapPoint{60.0_in, 17.0_deg},
      InterpMapPoint{120.0_in, 28.0_deg},  // Tested 2022-02.27, 50% accuracy
      InterpMapPoint{180.0_in, 30.0_deg},
      InterpMapPoint{240.0_in, 41.0_deg}};
}  // namespace shooterRange
namespace controllerMap {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array driveLongSpeed{InterpMapPoint{-1.0, -0.6},
                                                       InterpMapPoint{-0.75, -0.4},
                                                       InterpMapPoint{-0.15, 0.0},
                                                       InterpMapPoint{0.15, 0.0},
                                                       InterpMapPoint{0.75, 0.4},
                                                       InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array driveLatSpeed{InterpMapPoint{-1.0, -0.6},
                                                      InterpMapPoint{-0.75, -0.4},
                                                      InterpMapPoint{-0.15, 0.0},
                                                      InterpMapPoint{0.15, 0.0},
                                                      InterpMapPoint{0.75, 0.4},
                                                      InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array driveRotSpeed{
      InterpMapPoint{-1.0, -1.0}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 1.0}};

  [[maybe_unused]] constexpr std::array hookSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.2, 0.0}, InterpMapPoint{0.2, 0.0}, InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array armSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.2, 0.0}, InterpMapPoint{0.2, 0.0}, InterpMapPoint{1.0, 0.6}};

  [[maybe_unused]] constexpr std::array turretSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array hoodSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.6}};
}  // namespace controllerMap
