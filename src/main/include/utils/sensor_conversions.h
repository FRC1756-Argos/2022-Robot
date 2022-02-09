/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include "units/angle.h"
#include "units/length.h"

namespace sensor_conversions {
  namespace swerve_drive {
    namespace turn {
      constexpr double sensorConversionFactor =
          360.0 / 4096;  ///< multiply to convert raw sensor units to module degrees
      constexpr double ToSensorUnit(const units::degree_t degrees) {
        return degrees.to<double>() / sensorConversionFactor;
      }
      constexpr units::degree_t ToAngle(const double sensorunit) {
        return units::make_unit<units::degree_t>(sensorunit * sensorConversionFactor);
      }
    }  // namespace turn
  }    // namespace swerve_drive
  namespace turret {
    constexpr double sensorConversionFactor =
        4096.0 / 360;  ///< multiply to convert turret rotation degrees to raw sensor units
    constexpr double ToSensorUnit(const units::degree_t degrees) {
      return sensorConversionFactor * degrees.to<double>();
    }
    constexpr units::degree_t ToAngle(const double sensorunit) {
      return units::make_unit<units::degree_t>(sensorunit / sensorConversionFactor);
    }
  }  // namespace turret
  namespace hood {
    constexpr double sensorConversionFactor =
        360.0 / 4096;  ///< multiply to convert raw sensor units to motor output shaft degrees
    constexpr double sensorConversionDegTeeth =
        16.0 / 360;  ///< multiply to convert motor output shaft degrees to hood sproket teeth
    constexpr double sensorConversionTeethIn =
        4.0 / 16;  ///< multiply to convert hood sproket teeth to hood extension distance
    constexpr double sensorConversionInAngle =
        2 * M_PI / (360 * 11.5);  ///< multiply to convert hood extension distance to hood angle
    constexpr double ToSensorUnit(const units::degree_t degrees) {
      return degrees.to<double>() / sensorConversionInAngle / sensorConversionTeethIn / sensorConversionDegTeeth /
             sensorConversionFactor;
    }
    constexpr units::degree_t ToAngle(const double sensorunit) {
      return units::make_unit<units::degree_t>(sensorunit * sensorConversionFactor * sensorConversionDegTeeth *
                                               sensorConversionTeethIn * sensorConversionInAngle);
    }
  }  // namespace hood
  namespace climb_arms {
    constexpr double sensorToMotorRev = 1.0 / 2048;
    constexpr double gearboxReduction = 30.0 / 12;
    constexpr double extensionInchesPerRevolution = 1.0;  /// @todo How many threads per inch?
    constexpr units::inch_t ToExtension(const double sensorUnit) {
      return units::make_unit<units::inch_t>(sensorUnit * sensorToMotorRev * gearboxReduction *
                                             extensionInchesPerRevolution);
    }
    constexpr double ToSensorUnit(const units::inch_t extension) {
      return extension.to<double>() / extensionInchesPerRevolution / gearboxReduction / sensorToMotorRev;
    }
  }  // namespace climb_arms
  namespace climb_hooks {
    constexpr double sensorToMotorRev = 1.0 / 2048;
    constexpr double gearboxReduction = 1.0 / 8;
    constexpr double driveSprocketTeethPerRevolution = 16.0;  /// @todo Confirm this
    constexpr double extensionInchesPerDriveSprocketTooth = 0.25 / 1;
    constexpr units::inch_t ToExtension(const double sensorUnit) {
      return units::make_unit<units::inch_t>(sensorUnit * sensorToMotorRev * gearboxReduction *
                                             driveSprocketTeethPerRevolution * extensionInchesPerDriveSprocketTooth);
    }
    constexpr double ToSensorUnit(const units::inch_t extension) {
      return extension.to<double>() / extensionInchesPerDriveSprocketTooth / driveSprocketTeethPerRevolution /
             gearboxReduction / sensorToMotorRev;
    }
  }  // namespace climb_hooks
}  // namespace sensor_conversions
