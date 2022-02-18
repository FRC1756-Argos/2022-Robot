/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "addresses.h"
#include "argos_lib/config/status_frame_config.h"
#include "control_loops.h"
#include "ctre/Phoenix.h"
#include "units/voltage.h"

namespace motorConfig {
  namespace common {
    constexpr static auto neutralDeadband = 0.001;
    constexpr static auto voltCompSat = 11.0_V;
  }  // namespace common

  namespace drive {
    struct genericDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
    };
    struct frontLeftTurn {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
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
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
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
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
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
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
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
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
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
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::FollowerFX;
    };

    struct hoodMotor {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = true;
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
      constexpr static auto pid0_selectedSensor = ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
      constexpr static auto pid0_kP = controlLoop::shooter::hood::kP;
      constexpr static auto pid0_kI = controlLoop::shooter::hood::kI;
      constexpr static auto pid0_kD = controlLoop::shooter::hood::kD;
      constexpr static auto pid0_kF = controlLoop::shooter::hood::kF;
      constexpr static auto pid0_iZone = controlLoop::shooter::hood::iZone;
      constexpr static auto pid0_allowableError = controlLoop::shooter::hood::allowableError;
    };

    struct turretMotor {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
    };
  }  // namespace shooter

  namespace intake {
    struct beltDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
    };
    struct intakeDrive {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::None;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::Basic;
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
    };
    struct moveHook {
      constexpr static auto inverted = ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput;
      constexpr static bool sensorPhase = false;
      constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
      constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
      constexpr static auto voltCompSat = motorConfig::common::voltCompSat;
      constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::BasicFX;
    };
  }  // namespace climber
}  // namespace motorConfig
