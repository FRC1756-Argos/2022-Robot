/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/I2C.h>

namespace address {
  namespace drive {
    constexpr const char frontLeftDrive = 1;
    constexpr const char frontLeftTurn = 2;
    constexpr const char frontRightDrive = 3;
    constexpr const char frontRightTurn = 4;
    constexpr const char backRightDrive = 5;
    constexpr const char backRightTurn = 6;
    constexpr const char backLeftDrive = 7;
    constexpr const char backLeftTurn = 8;
  }  // namespace drive
  namespace climber {
    constexpr const char liftLeft = 9;
    constexpr const char liftRight = 10;
    constexpr const char moveHook = 11;
  }  // namespace climber
  namespace shooter {
    constexpr const char shooterWheelLeft = 12;
    constexpr const char shooterWheelRight = 13;
    constexpr const char hoodMotor = 14;
    constexpr const char turretMotor = 15;
  }  // namespace shooter
  namespace intake {
    constexpr const char beltDrive = 16;
    constexpr const char intakeDrive = 18;
  }  // namespace intake
  namespace encoders {
    constexpr const char frontLeftEncoder = 1;
    constexpr const char frontRightEncoder = 2;
    constexpr const char backRightEncoder = 3;
    constexpr const char backLeftEncoder = 4;
  }  // namespace encoders
  namespace controllers {
    constexpr const char driver = 0;
    constexpr const char secondary = 1;
  }  // namespace controllers
  namespace solenoids {
    constexpr const char intake = 0;
  }  // namespace solenoids
  namespace sensors {
    constexpr const char tofSensorIntake = 1;
    constexpr const char tofSensorShooter = 2;
    constexpr auto colorSensor = frc::I2C::Port::kOnboard;
  }  // namespace sensors

}  // namespace address
