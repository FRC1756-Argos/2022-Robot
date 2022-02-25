/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <array>

#include "argos_lib/general/interpolation.h"

namespace shooterRange {
  using argos_lib::InterpMapPoint;
  [[maybe_unused]] constexpr std::array shooterSpeed{InterpMapPoint{10.0, 2300.0},
                                                     InterpMapPoint{60.0, 2600.0},
                                                     InterpMapPoint{120.0, 3000.0},
                                                     InterpMapPoint{180.0, 3000.0},
                                                     InterpMapPoint{240.0, 3500.0}};
  [[maybe_unused]] constexpr std::array hoodAngle{InterpMapPoint{10.0, 10.0},
                                                  InterpMapPoint{60.0, 17.0},
                                                  InterpMapPoint{120.0, 25.0},
                                                  InterpMapPoint{180.0, 30.0},
                                                  InterpMapPoint{240.0, 41.0}};
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
