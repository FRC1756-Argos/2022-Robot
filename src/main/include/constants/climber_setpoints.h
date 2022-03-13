/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include <vector>

#include "constants/measure_up.h"
#include "utils/sensor_conversions.h"

struct ClimberPoint {
  units::inch_t hookExtension;
  units::inch_t armExtension;
  units::velocity::inches_per_second_t hookSpeed;
  units::velocity::inches_per_second_t armSpeed;
};

namespace ClimberSetpoints {
  constexpr ClimberPoint setup = {measure_up::climber_hook::maxExtension - 1_in, 37_in, 10_ips, 10_ips};
  constexpr ClimberPoint storage = {measure_up::climber_hook::maxExtension - 6_in, 21.5_in, 10_ips, 10_ips};
  constexpr ClimberPoint latchL2 = {28.7_in, 37_in, 15_ips, 15_ips};
  constexpr ClimberPoint prepL2 = {28.7_in, 35.8_in, 10_ips, 10_ips};
  constexpr ClimberPoint secureL2 = {1_in, 37_in, 15_ips, 15_ips};
  constexpr ClimberPoint passL3 = {34_in, 37.1_in, 10_ips, 10_ips};
  constexpr ClimberPoint latchL3 = {31.5_in, 37.1_in, 10_ips, 10_ips};
  constexpr ClimberPoint prepTransferL3 = {31.5_in, 29_in, 10_ips, 10_ips};
  constexpr ClimberPoint transferL3 = {26_in, 29_in, 10_ips, 10_ips};
}  // namespace ClimberSetpoints

// the sequence to get fed into climb command
namespace ClimberSequence {
  const std::vector<ClimberPoint> sequence = {ClimberSetpoints::latchL2, ClimberSetpoints::secureL2};
}  // namespace ClimberSequence
