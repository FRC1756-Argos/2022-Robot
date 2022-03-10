/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include "utils/sensor_conversions.h"

struct ClimberPoint {
  units::inch_t hookExtension;
  units::inch_t armExtension;
  units::velocity::inches_per_second_t hookSpeed;
  units::velocity::inches_per_second_t armSpeed;
};

namespace ClimberSetpoints {
  constexpr ClimberPoint setup = {34_in, 37_in, 10_ips, 10_ips};
  constexpr ClimberPoint latchL2 = {34_in, 28.7_in, 10_ips, 10_ips};
  constexpr ClimberPoint prepL2 = {28.7_in, 35.8_in, 10_ips, 10_ips};
  constexpr ClimberPoint secureL2 = {1_in, 35.8_in, 10_ips, 10_ips};
  constexpr ClimberPoint passL3 = {34_in, 37.1_in, 10_ips, 10_ips};
  constexpr ClimberPoint latchL3 = {31.5_in, 37.1_in, 10_ips, 10_ips};
  constexpr ClimberPoint prepTransferL3 = {31.5_in, 29_in, 10_ips, 10_ips};
  constexpr ClimberPoint transferL3 = {26_in, 29_in, 10_ips, 10_ips};
}  // namespace ClimberSetpoints
