/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"

namespace units {
  UNIT_ADD(velocity,
           inches_per_second,
           inches_per_second,
           ips,
           units::compound_unit<units::length::inches, units::inverse<units::time::second>>)
  UNIT_ADD(acceleration,
           inches_per_second_squared,
           inches_per_second_squared,
           ips2,
           units::compound_unit<units::length::inches, units::inverse<units::squared<units::time::second>>>)
}  // namespace units

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
    namespace drive {
      constexpr auto wheelDiameter = 4_in;
      constexpr auto wheelCircumference = wheelDiameter * M_PI;
      constexpr double sensorUnitsPerMotorRevolution = 2048;
      constexpr double driveGearRatio = 8.16;

      constexpr units::inch_t ToDistance(const double sensorunit) {
        return wheelCircumference * (sensorunit / sensorUnitsPerMotorRevolution / driveGearRatio);
      }
      constexpr double ToSensorPosition(const units::inch_t distance) {
        return (distance / wheelCircumference) * driveGearRatio * sensorUnitsPerMotorRevolution;
      }

      constexpr units::inches_per_second_t ToVelocity(const double sensorVelocity) {
        return ToDistance(sensorVelocity) / units::decisecond_t{1};
      }
      constexpr double ToSensorVelocity(const units::inches_per_second_t velocity) {
        return ToSensorPosition(velocity * units::decisecond_t{1});
      }
    }  // namespace drive
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
        360.0 / (2 * M_PI * 11.5);  ///< multiply to convert hood extension distance to hood angle
    constexpr double ToSensorUnit(const units::degree_t degrees) {
      return degrees.to<double>() / sensorConversionInAngle / sensorConversionTeethIn / sensorConversionDegTeeth /
             sensorConversionFactor;
    }
    constexpr units::degree_t ToAngle(const double sensorunit) {
      return units::make_unit<units::degree_t>(sensorunit * sensorConversionFactor * sensorConversionDegTeeth *
                                               sensorConversionTeethIn * sensorConversionInAngle);
    }
  }  // namespace hood
  namespace shooter {
    constexpr double sensorConversionFactor =
        10.0 * 60 / 2048;  ///< multiply to convert shooter velocity to revolutions per minute
    constexpr double ToSensorUnit(const units::revolutions_per_minute_t rpm) {
      return rpm.to<double>() / sensorConversionFactor;
    }
    constexpr units::revolutions_per_minute_t ToVelocity(const double sensorunit) {
      return units::make_unit<units::revolutions_per_minute_t>(sensorunit * sensorConversionFactor);
    }
  }  // namespace shooter
  namespace climb_arms {
    constexpr double sensorToMotorRev = 1.0 / 2048;
    constexpr double gearboxReduction = 10.0 / 30;
    constexpr double extensionMillimetersPerRevolution = 4.0;
    constexpr double fudgeFactor = 1.1;  ///< @todo Why are we off by this amount?
    constexpr units::inch_t ToExtension(const double sensorUnit) {
      return units::make_unit<units::millimeter_t>(sensorUnit * sensorToMotorRev * gearboxReduction *
                                                   extensionMillimetersPerRevolution * fudgeFactor);
    }
    constexpr double ToSensorUnit(const units::millimeter_t extension) {
      return extension.to<double>() / fudgeFactor / extensionMillimetersPerRevolution / gearboxReduction /
             sensorToMotorRev;
    }

    constexpr double ToSensorVelocity(const units::meters_per_second_t velocity) {
      return ToSensorUnit(velocity * units::decisecond_t{1});
    }

    constexpr units::meters_per_second_t ToVelocity(double sensorVelocity) {
      return units::meters_per_second_t{ToExtension(sensorVelocity) / units::decisecond_t{1}};
    }

    constexpr double ToSensorAccel(const units::meters_per_second_squared_t accel) {
      return ToSensorVelocity(accel * 1_s);
    }

    constexpr units::meters_per_second_squared_t ToAccel(double sensorAccel) { return ToVelocity(sensorAccel) / 1_s; }

  }  // namespace climb_arms
  namespace climb_hooks {
    constexpr double sensorToMotorRev = 1.0 / 2048;
    constexpr double gearboxReduction = 1.0 / 8;
    constexpr double driveSprocketTeethPerRevolution = 18.0;
    constexpr double sprocketRatio = 18.0 / 22.0;
    constexpr double extensionInchesPerDriveSprocketTooth = 0.25 / 1;
    constexpr double fudgeFactor = 0.88;  ///< @todo Why are we off by this amount?
    constexpr units::inch_t ToExtension(const double sensorUnit) {
      return units::make_unit<units::inch_t>(sensorUnit * sensorToMotorRev * gearboxReduction * sprocketRatio *
                                             driveSprocketTeethPerRevolution * extensionInchesPerDriveSprocketTooth *
                                             fudgeFactor);
    }
    constexpr double ToSensorUnit(const units::inch_t extension) {
      return extension.to<double>() / fudgeFactor / extensionInchesPerDriveSprocketTooth /
             driveSprocketTeethPerRevolution / gearboxReduction / sprocketRatio / sensorToMotorRev;
    }

    constexpr double ToSensorVelocity(const units::meters_per_second_t velocity) {
      return ToSensorUnit(velocity * units::decisecond_t{1});
    }

    constexpr units::meters_per_second_t ToVelocity(double sensorVelocity) {
      return units::meters_per_second_t{ToExtension(sensorVelocity) / units::decisecond_t{1}};
    }

    constexpr double ToSensorAccel(const units::meters_per_second_squared_t accel) {
      return ToSensorVelocity(accel * 1_s);
    }

    constexpr units::meters_per_second_squared_t ToAccel(double sensorAccel) { return ToVelocity(sensorAccel) / 1_s; }
  }  // namespace climb_hooks
}  // namespace sensor_conversions
