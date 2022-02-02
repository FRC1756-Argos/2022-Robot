/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include "units/Angle.h"

namespace sensor_conversions {
  namespace turret {
    constexpr double sensorConversionFactor = 4096.0 / 360;  ///< multiply to convert turret rotation degrees to raw sensor units
    constexpr double ToSensorUnit(const units::degree_t degrees) {
      return sensorConversionFactor * degrees.to<double>();
    }
    constexpr units::degree_t ToAngle(const double sensorunit) {
      return units::make_unit<units::degree_t>(sensorunit / sensorConversionFactor);
    }
  }  // namespace turret
  namespace hood {
    constexpr double sensorConversionFactor = 360.0 / 4096;  ///< multiply to convert raw sensor units to motor output shaft degrees
    constexpr double sensorConversionDegTeeth = 16.0 / 360;  ///< multiply to convert motor output shaft degrees to hood sproket teeth
    constexpr double sensorConversionTeethIn = 4.0 / 16;     ///< multiply to convert hood sproket teeth to hood extension distance
    constexpr double sensorConversionInAngle = 2 * M_PI / (360 * 11.5); ///< multiply to convert hood extension distance to hood angle
    constexpr double ToSensorUnit(const units::degree_t degrees) {
      return degrees.to<double>() / sensorConversionInAngle / sensorConversionTeethIn / sensorConversionDegTeeth /
             sensorConversionFactor;
    }
    constexpr units::degree_t ToAngle(const double sensorunit) {
      return units::make_unit<units::degree_t>(sensorunit * sensorConversionFactor * sensorConversionDegTeeth *
                                               sensorConversionTeethIn * sensorConversionInAngle);
    }
  }  // namespace hood
}  // namespace sensor_conversions
