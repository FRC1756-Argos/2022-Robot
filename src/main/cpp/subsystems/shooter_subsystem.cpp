/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/shooter_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "argos_lib/config/talonsrx_config.h"
#include "units/length.h"
#include "utils/sensor_conversions.h"

ShooterSubsystem::ShooterSubsystem()
    : m_shooterWheelLeft(address::shooter::shooterWheelLeft)
    , m_shooterWheelRight(address::shooter::shooterWheelRight)
    , m_angleControl(address::shooter::angleControl)
    , m_rotationControl(address::shooter::rotationControl)
    , m_cameraInterface()
    , m_hoodHomed(false)
    , m_manualOverride(false)
    , m_hoodPIDTuner{"argos/hood",
                     {&m_angleControl},
                     0,
                     argos_lib::ClosedLoopSensorConversions{
                         argos_lib::GetSensorConversionFactor(sensor_conversions::hood::ToAngle),
                         1.0,
                         argos_lib::GetSensorConversionFactor(sensor_conversions::hood::ToAngle)}}
    , m_shooterPIDTuner{"argos/shooter",
                        {&m_shooterWheelLeft},
                        0,
                        argos_lib::ClosedLoopSensorConversions{
                            1.0,
                            argos_lib::GetSensorConversionFactor(sensor_conversions::shooter::ToVelocity),
                            argos_lib::GetSensorConversionFactor(sensor_conversions::shooter::ToVelocity)}}
    , m_turretPIDTuner{"argos/turret",
                       {&m_rotationControl},
                       0,
                       argos_lib::ClosedLoopSensorConversions{
                           argos_lib::GetSensorConversionFactor(sensor_conversions::turret::ToAngle),
                           1.0,
                           argos_lib::GetSensorConversionFactor(sensor_conversions::turret::ToAngle)}} {
  argos_lib::falcon_config::FalconConfig<motorConfig::shooter::shooterWheelLeft>(m_shooterWheelLeft, 50_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::shooter::shooterWheelRight>(m_shooterWheelRight, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::shooter::angleControl>(m_angleControl, 50_ms);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::shooter::rotationControl>(m_rotationControl, 50_ms);

  m_angleControl.SetSelectedSensorPosition(sensor_conversions::hood::ToSensorUnit(0_deg));

  m_shooterWheelRight.Follow(m_shooterWheelLeft);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::AutoAim() {}

void ShooterSubsystem::Shoot(double ballfiringspeed) {
  m_shooterWheelLeft.Set(ballfiringspeed);
}

void ShooterSubsystem::CloseLoopShoot(units::revolutions_per_minute_t ShooterWheelSpeed) {
  m_shooterWheelLeft.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity,
                         sensor_conversions::shooter::ToSensorUnit(ShooterWheelSpeed));
}

void ShooterSubsystem::ManualAim(double turnSpeed, double hoodSpeed) {
  if (turnSpeed != 0 || hoodSpeed != 0) {
    m_manualOverride = true;
  }
  if (m_manualOverride) {
    MoveTurret(turnSpeed);
    MoveHood(hoodSpeed);
  }
}

void ShooterSubsystem::MoveHood(double hoodSpeed) {
  m_angleControl.Set(hoodSpeed);
}

void ShooterSubsystem::MoveTurret(double turnSpeed) {
  m_rotationControl.Set(turnSpeed);
}

void ShooterSubsystem::HoodSetPosition(units::degree_t angle) {
  if (IsHoodHomed()) {
    m_manualOverride = false;
    angle *= -1;
    m_angleControl.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                       sensor_conversions::hood::ToSensorUnit(angle));
  }
}

void ShooterSubsystem::UpdateHoodHome() {
  m_angleControl.SetSelectedSensorPosition(sensor_conversions::hood::ToSensorUnit(measure_up::hood::homeAngle));
  m_hoodHomed = true;
}

bool ShooterSubsystem::IsHoodMoving() {
  return std::fabs(m_angleControl.GetSelectedSensorVelocity()) > 5.0;
}

bool ShooterSubsystem::IsHoodHomed() {
  return m_hoodHomed;
}

bool ShooterSubsystem::IsManualOverride() {
  return m_manualOverride;
}

void ShooterSubsystem::Disable() {
  m_manualOverride = false;
}

/// @todo change math
units::inch_t ShooterSubsystem::GetTargetDistance(units::degree_t targetVerticalAngle) {
  return (measure_up::camera::upperHubHeight - measure_up::camera::cameraHeight) *
         std::tan(
             static_cast<units::radian_t>(measure_up::camera::cameraMountAngle + targetVerticalAngle).to<double>());
}

// CAMERA INTERFACE -----------------------------------------------------------------------------
CameraInterface::CameraInterface() : m_camera{camera::nickname} {
  // SETS DEFAULT PIPELINE
  m_camera.SetPipelineIndex(camera::defaultPipelineIndex);
}

std::optional<photonlib::PhotonTrackedTarget> CameraInterface::GetHighestTarget() {
  // GET THE MOST RECENT RESULT
  photonlib::PhotonPipelineResult latestResult = m_camera.GetLatestResult();

  // CHECK IF NO TARGETS
  if (!latestResult.HasTargets()) {
    return std::nullopt;
  }

  // GET TARGETS FROM RESULT
  const wpi::span<const photonlib::PhotonTrackedTarget> targets = latestResult.GetTargets();

  // FIND HIGHEST TARGET
  return *std::max_element(targets.begin(),
                           targets.end(),
                           [](const photonlib::PhotonTrackedTarget& lhs, const photonlib::PhotonTrackedTarget& rhs) {
                             return lhs.GetPitch() < rhs.GetPitch();
                           });
}

/// @todo would we need to use "this->" here to reference the object that called?
void CameraInterface::SetDriverMode(bool mode) {
  m_camera.SetDriverMode(mode);
}

units::degree_t ShooterSubsystem::GetTurretTarget(photonlib::PhotonTrackedTarget target) {
  double targetDistance = GetTargetDistance(units::make_unit<units::degree_t>(target.GetPitch())).to<double>();
  double x = measure_up::camera::toRotationCenter.to<double>();
  // calculate d2
  double d2 = std::sqrt(std::pow(x, 2.0) + std::pow(targetDistance, 2.0));

  /// @todo valid conversion?
  double currentAngle =
      m_rotationControl.GetSelectedSensorPosition() * sensor_conversions::shooter::sensorConversionFactor;

  units::degree_t targetAngle = units::make_unit<units::degree_t>(
      currentAngle + std::acos((std::pow(x, 2.0) + d2 - targetDistance) / 2 * x * d2));

  return targetAngle;
}
