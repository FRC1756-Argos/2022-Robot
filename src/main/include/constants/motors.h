/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "addresses.h"
#include "argos_lib/config/status_frame_config.h"
#include "control_loops.h"
#include "ctre/Phoenix.h"
#include "units/current.h"
#include "units/time.h"
#include "units/voltage.h"

namespace motorConfig {
  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configuration settings shared by all robot configurations
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace common {
    constexpr static auto neutralDeadband = 0.001;
    constexpr static auto voltCompSat = 11.0_V;
  }  // namespace common

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to competition robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace comp_bot {
    namespace drive {
      struct genericDrive {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::drive::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::drive::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::drive::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::drive::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::drive::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::drive::allowableError;
      };
      struct frontLeftTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::frontLeftEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };
      struct frontRightTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::frontRightEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };
      struct backRightTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::backRightEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };
      struct backLeftTurn {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::backLeftEncoder;
        constexpr static auto remoteFilter0_type =
            ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::drive::rotate::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::drive::rotate::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::drive::rotate::allowableError;
      };

    }  // namespace drive

    namespace shooter {
      struct shooterWheelLeft {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::shooter::shooter::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::shooter::shooter::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::shooter::shooter::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::shooter::shooter::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::shooter::shooter::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::shooter::shooter::allowableError;
      };

      struct shooterWheelRight {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::FollowerFX;
      };

      struct hoodMotor {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
        constexpr static auto peakCurrentLimit = 30_A;
        constexpr static auto peakCurrentDuration = 1_s;
        constexpr static auto continuousCurrentLimit = 20_A;
        constexpr static auto pid0_selectedSensor =
            ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
        constexpr static auto pid0_kP = controlLoop::comp_bot::shooter::hood::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::shooter::hood::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::shooter::hood::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::shooter::hood::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::shooter::hood::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::shooter::hood::allowableError;
      };

      struct turretMotor {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
        constexpr static auto peakCurrentLimit = 30_A;
        constexpr static auto peakCurrentDuration = 1_s;
        constexpr static auto continuousCurrentLimit = 20_A;
        constexpr static auto peakOutputForward = 0.7;
        constexpr static auto peakOutputReverse = -0.7;
        constexpr static auto pid0_selectedSensor =
            ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
        constexpr static auto pid0_kP = controlLoop::comp_bot::shooter::turret::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::shooter::turret::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::shooter::turret::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::shooter::turret::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::shooter::turret::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::shooter::turret::allowableError;
      };
    }  // namespace shooter

    namespace intake {
      struct beltDrive {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
        constexpr static auto peakCurrentLimit = 30_A;
        constexpr static auto peakCurrentDuration = 1_s;
        constexpr static auto continuousCurrentLimit = 20_A;
      };
      struct intakeDrive {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
        constexpr static auto peakCurrentLimit = 30_A;
        constexpr static auto peakCurrentDuration = 1_s;
        constexpr static auto continuousCurrentLimit = 20_A;
      };

    }  // namespace intake
    namespace climber {
      struct liftRight {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
        constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
        constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
        constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
        constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::climber::arm::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::climber::arm::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::climber::arm::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::climber::arm::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::climber::arm::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::climber::arm::allowableError;
        constexpr static auto pid1_kP = controlLoop::comp_bot::climber_loaded::arm::kP;
        constexpr static auto pid1_kI = controlLoop::comp_bot::climber_loaded::arm::kI;
        constexpr static auto pid1_kD = controlLoop::comp_bot::climber_loaded::arm::kD;
        constexpr static auto pid1_kF = controlLoop::comp_bot::climber_loaded::arm::kF;
        constexpr static auto pid1_iZone = controlLoop::comp_bot::climber_loaded::arm::iZone;
        constexpr static auto pid1_allowableError = controlLoop::comp_bot::climber_loaded::arm::allowableError;
      };
      struct liftLeft {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
        constexpr static auto forwardLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
        constexpr static auto forwardLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
        constexpr static auto reverseLimit_normalState = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
        constexpr static auto reverseLimit_source = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::climber::arm::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::climber::arm::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::climber::arm::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::climber::arm::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::climber::arm::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::climber::arm::allowableError;
        constexpr static auto pid1_kP = controlLoop::comp_bot::climber_loaded::arm::kP;
        constexpr static auto pid1_kI = controlLoop::comp_bot::climber_loaded::arm::kI;
        constexpr static auto pid1_kD = controlLoop::comp_bot::climber_loaded::arm::kD;
        constexpr static auto pid1_kF = controlLoop::comp_bot::climber_loaded::arm::kF;
        constexpr static auto pid1_iZone = controlLoop::comp_bot::climber_loaded::arm::iZone;
        constexpr static auto pid1_allowableError = controlLoop::comp_bot::climber_loaded::arm::allowableError;
      };
      struct moveHook {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
        constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::climber::hook::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::climber::hook::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::climber::hook::kD;
        constexpr static auto pid0_kF = controlLoop::comp_bot::climber::hook::kF;
        constexpr static auto pid0_iZone = controlLoop::comp_bot::climber::hook::iZone;
        constexpr static auto pid0_allowableError = controlLoop::comp_bot::climber::hook::allowableError;
        constexpr static auto pid1_kP = controlLoop::comp_bot::climber_loaded::hook::kP;
        constexpr static auto pid1_kI = controlLoop::comp_bot::climber_loaded::hook::kI;
        constexpr static auto pid1_kD = controlLoop::comp_bot::climber_loaded::hook::kD;
        constexpr static auto pid1_kF = controlLoop::comp_bot::climber_loaded::hook::kF;
        constexpr static auto pid1_iZone = controlLoop::comp_bot::climber_loaded::hook::iZone;
        constexpr static auto pid1_allowableError = controlLoop::comp_bot::climber_loaded::hook::allowableError;
      };
    }  // namespace climber
  }    // namespace comp_bot

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to practice robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace practice_bot {
    namespace drive {
      using genericDrive = motorConfig::comp_bot::drive::genericDrive;

      struct frontLeftTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::frontLeftTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::frontLeftTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::frontLeftTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::frontLeftTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::frontLeftTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::frontLeftTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::frontLeftEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::frontLeftTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::frontLeftTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::frontLeftTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::frontLeftTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::frontLeftTurn::pid0_allowableError;
      };
      struct frontRightTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::frontRightTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::frontRightTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::frontRightTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::frontRightTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::frontRightTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::frontRightTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::frontRightEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::frontRightTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::frontRightTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::frontRightTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::frontRightTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::frontRightTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::frontRightTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::frontRightTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::frontRightTurn::pid0_allowableError;
      };
      struct backRightTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::backRightTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::backRightTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::backRightTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::backRightTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::backRightTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::backRightTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::backRightEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::backRightTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::backRightTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::backRightTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::backRightTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::backRightTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::backRightTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::backRightTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::backRightTurn::pid0_allowableError;
      };
      struct backLeftTurn {
        constexpr static auto inverted = motorConfig::comp_bot::drive::backLeftTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::comp_bot::drive::backLeftTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::comp_bot::drive::backLeftTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::comp_bot::drive::backLeftTurn::neutralMode;
        constexpr static auto voltCompSat = motorConfig::comp_bot::drive::backLeftTurn::voltCompSat;
        constexpr static auto statusFrameMotorMode = motorConfig::comp_bot::drive::backLeftTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::backLeftEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::comp_bot::drive::backLeftTurn::remoteFilter0_type;
        constexpr static auto pid0_selectedSensor = motorConfig::comp_bot::drive::backLeftTurn::pid0_selectedSensor;
        constexpr static auto pid0_kP = motorConfig::comp_bot::drive::backLeftTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::comp_bot::drive::backLeftTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::comp_bot::drive::backLeftTurn::pid0_kD;
        constexpr static auto pid0_kF = motorConfig::comp_bot::drive::backLeftTurn::pid0_kF;
        constexpr static auto pid0_iZone = motorConfig::comp_bot::drive::backLeftTurn::pid0_iZone;
        constexpr static auto pid0_allowableError = motorConfig::comp_bot::drive::backLeftTurn::pid0_allowableError;
      };
    }  // namespace drive

    namespace shooter {
      using shooterWheelLeft = motorConfig::comp_bot::shooter::shooterWheelLeft;
      using shooterWheelRight = motorConfig::comp_bot::shooter::shooterWheelRight;
      struct hoodMotor {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
        constexpr static auto peakCurrentLimit = 30_A;
        constexpr static auto peakCurrentDuration = 1_s;
        constexpr static auto continuousCurrentLimit = 20_A;
        constexpr static auto pid0_selectedSensor =
            ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
        constexpr static auto pid0_kP = controlLoop::practice_bot::shooter::hood::kP;
        constexpr static auto pid0_kI = controlLoop::practice_bot::shooter::hood::kI;
        constexpr static auto pid0_kD = controlLoop::practice_bot::shooter::hood::kD;
        constexpr static auto pid0_kF = controlLoop::practice_bot::shooter::hood::kF;
        constexpr static auto pid0_iZone = controlLoop::practice_bot::shooter::hood::iZone;
        constexpr static auto pid0_allowableError = controlLoop::practice_bot::shooter::hood::allowableError;
      };
      struct turretMotor {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
        constexpr static auto peakCurrentLimit = 30_A;
        constexpr static auto peakCurrentDuration = 1_s;
        constexpr static auto continuousCurrentLimit = 20_A;
        constexpr static auto peakOutputForward = 1;
        constexpr static auto peakOutputReverse = -1;
        constexpr static auto pid0_selectedSensor =
            ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
        constexpr static auto pid0_kP = controlLoop::practice_bot::shooter::turret::kP;
        constexpr static auto pid0_kI = controlLoop::practice_bot::shooter::turret::kI;
        constexpr static auto pid0_kD = controlLoop::practice_bot::shooter::turret::kD;
        constexpr static auto pid0_kF = controlLoop::practice_bot::shooter::turret::kF;
        constexpr static auto pid0_iZone = controlLoop::practice_bot::shooter::turret::iZone;
        constexpr static auto pid0_allowableError = controlLoop::practice_bot::shooter::turret::allowableError;
      };
    }  // namespace shooter

    namespace intake {
      struct beltDrive {
        constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
        constexpr static bool sensorPhase = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
        constexpr static auto peakCurrentLimit = 30_A;
        constexpr static auto peakCurrentDuration = 1_s;
        constexpr static auto continuousCurrentLimit = 20_A;
      };
      using intakeDrive = motorConfig::comp_bot::intake::intakeDrive;
    }  // namespace intake
    namespace climber {
      using liftRight = motorConfig::comp_bot::climber::liftRight;
      using liftLeft = motorConfig::comp_bot::climber::liftLeft;
      using moveHook = motorConfig::comp_bot::climber::moveHook;
    }  // namespace climber
  }    // namespace practice_bot
}  // namespace motorConfig
