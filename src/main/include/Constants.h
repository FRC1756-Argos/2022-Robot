/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include <string>

#include "constants/addresses.h"
#include "constants/control_loops.h"
#include "constants/interpolation_maps.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace threshholds {
  namespace intake {
    const auto intakeDeactivate = 8_in;
    const auto intakeActivate = 8_in;
  }  // namespace intake
  namespace shooter {
    constexpr auto acceptableTurretError = 0.5_deg;
    constexpr auto acceptableHoodError = 0.5_deg;
    constexpr auto acceptableWheelError = 100_rpm;
    constexpr auto acceptableRangeTime = 250_ms;
  }  // namespace shooter
}  // namespace threshholds

namespace speeds {
  namespace intake {
    constexpr double beltForwardIntake = 0.4;
    constexpr double beltForwardShoot = 0.65;
    constexpr double beltReverse = -0.8;
    constexpr double intakeForward = 0.5;
    constexpr double intakeReverse = -1;
  }  // namespace intake
  namespace drive {
    constexpr units::velocity::feet_per_second_t maxAngular = 12_fps;
  }  // namespace drive
}  // namespace speeds

namespace pneumatics {
  namespace directions {
    constexpr bool intakeExtend = true;
    constexpr bool intakeRetract = false;
  }  // namespace directions
}  // namespace pneumatics

namespace camera {
  constexpr char targetingPipeline = 0;
  constexpr char driverPipeline = 1;
  constexpr int horizontalPixelResolution = 320;
  constexpr int verticalPixelResolution = 240;
  constexpr int horizontalAngleResolution = 54;
  constexpr int verticalAngleResolution = 41;
}  // namespace camera

namespace indexes {
  namespace swerveModules {
    constexpr char frontLeftIndex = 0;
    constexpr char frontRightIndex = 1;
    constexpr char backRightIndex = 2;
    constexpr char backLeftIndex = 3;
  }  // namespace swerveModules
}  // namespace indexes

namespace paths {
  const std::string swerveHomesPath = "homes/swerveHomes";
  const std::string turretHomePath = "homes/turretHome";
}  // namespace paths

namespace networkTables {
  namespace swerveHomes {
    const std::string tableKey = "Argos";
    namespace keys {
      const std::string flHome = "swerveHomes/flHome";
      const std::string frHome = "swerveHomes/frHome";
      const std::string brHome = "swerveHomes/brHome";
      const std::string blHome = "swerveHomes/blHome";

      const std::string flHomeFullPath = "swerveHomes/flHome";
      const std::string frHomeFullPath = "swerveHomes/frHome";
      const std::string brHomeFullPath = "swerveHomes/brHome";
      const std::string blHomeFullPath = "swerveHomes/blHome";
    }  // namespace keys
  }    // namespace swerveHomes
}  // namespace networkTables
