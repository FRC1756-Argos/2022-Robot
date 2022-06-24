/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/I2C.h>

#include "argos_lib/config/config_types.h"

namespace address {
  namespace comp_bot {
    struct drive {
      constexpr static argos_lib::CANAddress frontLeftDrive{1, "rio"};
      constexpr static argos_lib::CANAddress frontLeftTurn{2, "rio"};
      constexpr static argos_lib::CANAddress frontRightDrive{3, "rio"};
      constexpr static argos_lib::CANAddress frontRightTurn{4, "rio"};
      constexpr static argos_lib::CANAddress backRightDrive{5, "rio"};
      constexpr static argos_lib::CANAddress backRightTurn{6, "rio"};
      constexpr static argos_lib::CANAddress backLeftDrive{7, "rio"};
      constexpr static argos_lib::CANAddress backLeftTurn{8, "rio"};
    };
    struct climber {
      constexpr static argos_lib::CANAddress liftLeft{9, "rio"};
      constexpr static argos_lib::CANAddress liftRight{10, "rio"};
      constexpr static argos_lib::CANAddress moveHook{11, "rio"};
    };
    struct shooter {
      constexpr static argos_lib::CANAddress shooterWheelLeft{12, "rio"};
      constexpr static argos_lib::CANAddress shooterWheelRight{13, "rio"};
      constexpr static argos_lib::CANAddress hoodMotor{14, "rio"};
      constexpr static argos_lib::CANAddress turretMotor{15, "rio"};
    };
    struct intake {
      constexpr static argos_lib::CANAddress beltDrive{16, "rio"};
      constexpr static argos_lib::CANAddress intakeDrive{18, "rio"};
    };
    struct encoders {
      constexpr static argos_lib::CANAddress frontLeftEncoder{1, "rio"};
      constexpr static argos_lib::CANAddress frontRightEncoder{2, "rio"};
      constexpr static argos_lib::CANAddress backRightEncoder{3, "rio"};
      constexpr static argos_lib::CANAddress backLeftEncoder{4, "rio"};
    };
    struct controllers {
      constexpr static const char driver = 0;
      constexpr static const char secondary = 1;
    };
    struct solenoids {
      constexpr static const char intake = 0;
    };
    struct sensors {
      constexpr static const char tofSensorIntake = 1;
      constexpr static const char tofSensorShooter = 2;
      constexpr static auto colorSensor = frc::I2C::Port::kOnboard;
      constexpr static argos_lib::CANAddress pigeonIMU{1, "drive"};
    };
  }  // namespace comp_bot
  namespace practice_bot {
    namespace drive {
      constexpr static argos_lib::CANAddress frontLeftDrive{address::comp_bot::drive::frontLeftDrive.address, "drive"};
      constexpr static argos_lib::CANAddress frontLeftTurn{address::comp_bot::drive::frontLeftTurn.address, "drive"};
      constexpr static argos_lib::CANAddress frontRightDrive{address::comp_bot::drive::frontRightDrive.address,
                                                             "drive"};
      constexpr static argos_lib::CANAddress frontRightTurn{address::comp_bot::drive::frontRightTurn.address, "drive"};
      constexpr static argos_lib::CANAddress backRightDrive{address::comp_bot::drive::backRightDrive.address, "drive"};
      constexpr static argos_lib::CANAddress backRightTurn{address::comp_bot::drive::backRightTurn.address, "drive"};
      constexpr static argos_lib::CANAddress backLeftDrive{address::comp_bot::drive::backLeftDrive.address, "drive"};
      constexpr static argos_lib::CANAddress backLeftTurn{address::comp_bot::drive::backLeftTurn.address, "drive"};
    }  // namespace drive
    using climber = address::comp_bot::climber;
    using shooter = address::comp_bot::shooter;
    using intake = address::comp_bot::intake;
    namespace encoders {
      constexpr static argos_lib::CANAddress frontLeftEncoder{address::comp_bot::encoders::frontLeftEncoder.address,
                                                              "drive"};
      constexpr static argos_lib::CANAddress frontRightEncoder{address::comp_bot::encoders::frontRightEncoder.address,
                                                               "drive"};
      constexpr static argos_lib::CANAddress backRightEncoder{address::comp_bot::encoders::backRightEncoder.address,
                                                              "drive"};
      constexpr static argos_lib::CANAddress backLeftEncoder{address::comp_bot::encoders::backLeftEncoder.address,
                                                             "drive"};
    }  // namespace encoders
    using controllers = address::comp_bot::controllers;
    using solenoids = address::comp_bot::solenoids;
    using sensors = address::comp_bot::sensors;
  }  // namespace practice_bot

}  // namespace address
