#pragma once
#include "units/Angle.h"
namespace sensor_conversions{
  namespace turret{
    constexpr double sensorConversionFactor=4096.0/360; ///< sensor units per degree
   constexpr double ToSensorUnit(const units::degree_t degrees){
     return sensorConversionFactor*degrees.to<double>();
   }
   constexpr units::degree_t ToAngle (const double sensorunit){
     return units::make_unit<units::degree_t>(sensorunit/sensorConversionFactor);
   }
  }
}
