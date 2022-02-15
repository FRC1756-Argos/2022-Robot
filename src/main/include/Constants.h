/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/I2C.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <string>

#include "argos_lib/general/interpolation.h"
#include "ctre/Phoenix.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace speeds {
  namespace intake {
    constexpr double beltForward = -1;
    constexpr double beltReverse = 0.8;
    constexpr double intakeForward = 1;
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
    constexpr const char angleControl = 14;
    constexpr const char rotationControl = 15;
  }  // namespace shooter
  namespace intake {
    constexpr const char beltDrive = 16;
    constexpr const char elevatorIntakeDrive = 17;
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

namespace measure_up {
  namespace chassis {
    constexpr units::inch_t width{28.0};
    constexpr units::inch_t length{31.0};
  }  // namespace chassis
  namespace swerve_offsets {
    constexpr auto frontLeftLOffset = 4.0_in;
    constexpr auto frontLeftWOffset = 4.0_in;
    constexpr auto frontRightLOffset = 4.0_in;
    constexpr auto frontRightWOffset = 4.0_in;
    constexpr auto backRightWOffset = 4.0_in;
    constexpr auto backRightLOffset = 4.0_in;
    constexpr auto backLeftWOffset = 4.0_in;
    constexpr auto backLeftLOffset = 4.0_in;
  }  // namespace swerve_offsets
  namespace hood {
    constexpr auto homeAngle = 0_deg;  ///< Ain't that a nice design feature :)
  }                                    // namespace hood
}  // namespace measure_up

namespace camera {
  /// @todo replace with camera nickname
  const std::string nickname = "photonvision";
  constexpr char defaultPipelineIndex = 0;
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

namespace controllerMap {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array driveLongSpeed{InterpMapPoint{-1.0, -0.6},
                                                       InterpMapPoint{-0.75, -0.4},
                                                       InterpMapPoint{-0.15, 0.0},
                                                       InterpMapPoint{0.15, 0.0},
                                                       InterpMapPoint{0.75, 0.4},
                                                       InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array driveLatSpeed{InterpMapPoint{-1.0, -0.6},
                                                      InterpMapPoint{-0.75, -0.4},
                                                      InterpMapPoint{-0.15, 0.0},
                                                      InterpMapPoint{0.15, 0.0},
                                                      InterpMapPoint{0.75, 0.4},
                                                      InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array driveRotSpeed{
      InterpMapPoint{-1.0, -1.0}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 1.0}};

  [[maybe_unused]] constexpr std::array hookSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.2, 0.0}, InterpMapPoint{0.2, 0.0}, InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array armSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.2, 0.0}, InterpMapPoint{0.2, 0.0}, InterpMapPoint{1.0, 0.6}};

  [[maybe_unused]] constexpr std::array turretSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.6}};
  [[maybe_unused]] constexpr std::array hoodSpeed{
      InterpMapPoint{-1.0, -0.6}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.6}};
}  // namespace controllerMap

namespace controlLoop {
  namespace drive {
    namespace rotate {
      constexpr double kP = 1.4;
      constexpr double kI = 0.0005;
      constexpr double kD = 0.0;
      constexpr double kF = 0.0;
      constexpr double iZone = 500.0;
      constexpr double allowableError = 0.0;
    }  // namespace rotate
  }    // namespace drive
  namespace shooter {
    namespace shooter {
      constexpr double kP = 0.01;
      constexpr double kI = 0.0;
      constexpr double kD = 0.0;
      constexpr double kF = 0.05;
      constexpr double iZone = 100.0;
      constexpr double allowableError = 0.0;
    }  // namespace shooter
    namespace hood {
      constexpr double kP = 0.25;
      constexpr double kI = 0.01;
      constexpr double kD = 0.01;
      constexpr double kF = 0.0;
      constexpr double iZone = 600.0;
      constexpr double allowableError = 0.0;
    }  // namespace hood
  }    // namespace shooter
}  // namespace controlLoop

namespace motorConfig {
  namespace drive {
    struct genericDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
    struct frontLeftTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoders::frontLeftEncoder;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };
    struct frontRightTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoders::frontRightEncoder;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };
    struct backRightTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoders::backRightEncoder;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };
    struct backLeftTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto remoteFilter0_addr = address::encoders::backLeftEncoder;
      constexpr static auto remoteFilter0_type =
          ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      constexpr static auto pid0_kP = controlLoop::drive::rotate::kP;
      constexpr static auto pid0_kI = controlLoop::drive::rotate::kI;
      constexpr static auto pid0_kD = controlLoop::drive::rotate::kD;
      constexpr static auto pid0_kF = controlLoop::drive::rotate::kF;
      constexpr static auto pid0_iZone = controlLoop::drive::rotate::iZone;
      constexpr static auto pid0_allowableError = controlLoop::drive::rotate::allowableError;
    };

  }  // namespace drive

  namespace shooter {
    struct shooterWheelLeft {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
      constexpr static auto pid0_kP = controlLoop::shooter::shooter::kP;
      constexpr static auto pid0_kI = controlLoop::shooter::shooter::kI;
      constexpr static auto pid0_kD = controlLoop::shooter::shooter::kD;
      constexpr static auto pid0_kF = controlLoop::shooter::shooter::kF;
      constexpr static auto pid0_iZone = controlLoop::shooter::shooter::iZone;
      constexpr static auto pid0_allowableError = controlLoop::shooter::shooter::allowableError;
    };

    struct shooterWheelRight {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = 11.0_V;
    };

    struct angleControl {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = true;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
      constexpr static auto pid0_kP = controlLoop::shooter::hood::kP;
      constexpr static auto pid0_kI = controlLoop::shooter::hood::kI;
      constexpr static auto pid0_kD = controlLoop::shooter::hood::kD;
      constexpr static auto pid0_kF = controlLoop::shooter::hood::kF;
      constexpr static auto pid0_iZone = controlLoop::shooter::hood::iZone;
      constexpr static auto pid0_allowableError = controlLoop::shooter::hood::allowableError;
    };

    struct rotationControl {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = 11.0_V;
    };
  }  // namespace shooter

  namespace intake {
    struct beltDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
    struct elevatorIntakeDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
    struct intakeDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };

  }  // namespace intake
  namespace climber {
    struct liftRight {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
    };
    struct liftLeft {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
      constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
      constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
      constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
    };
    struct moveHook {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = 0.001;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = 11.0_V;
    };
  }  // namespace climber
}  // namespace motorConfig
