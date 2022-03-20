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
  char pidSlot;
};

namespace ClimberSetpoints {
  constexpr ClimberPoint setup = {measure_up::climber_hook::maxExtension - 1_in, 37_in, 25_ips, 25_ips, 0};
  constexpr ClimberPoint storage = {measure_up::climber_hook::maxExtension - 6_in, 21.5_in, 25_ips, 25_ips, 0};
  constexpr ClimberPoint latchL2 = {30.75_in, 37_in, 17_ips, 17_ips, 0};
  constexpr ClimberPoint prepL2 = {28.7_in, 35.8_in, 10_ips, 10_ips, 0};
  constexpr ClimberPoint secureL2 = {1_in, 37_in, 18_ips, 18_ips, 1};
  constexpr ClimberPoint passL3 = {34.4_in, 31.1_in, 20_ips, 20_ips, 0};
  constexpr ClimberPoint latchL3 = {31.5_in, 37.1_in, 10_ips, 10_ips, 0};
  constexpr ClimberPoint prepTransferL3 = {31.5_in, 29_in, 10_ips, 10_ips, 0};
  constexpr ClimberPoint transferL3 = {26_in, 29_in, 10_ips, 10_ips, 0};
  constexpr ClimberPoint holdL3 = {26_in, 31.1_in, 17_ips, 25_ips, 1};
}  // namespace ClimberSetpoints

// the sequence to get fed into climb command
namespace ClimberSequence {
  const std::vector<ClimberPoint> sequence = {
      ClimberSetpoints::secureL2, ClimberSetpoints::passL3, ClimberSetpoints::holdL3};
}  // namespace ClimberSequence
