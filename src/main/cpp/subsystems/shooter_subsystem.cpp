/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/shooter_subsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "argos_lib/config/talonsrx_config.h"
#include "argos_lib/general/swerve_utils.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/length.h"
#include "utils/sensor_conversions.h"

ShooterSubsystem::ShooterSubsystem(const argos_lib::RobotInstance instance)
    : m_shooterWheelLeft(address::shooter::shooterWheelLeft)
    , m_shooterWheelRight(address::shooter::shooterWheelRight)
    , m_hoodMotor(address::shooter::hoodMotor)
    , m_turretMotor(address::shooter::turretMotor)
    , m_cameraInterface()
    , m_turretHomingStorage(paths::turretHomePath)
    , m_hoodHomed(false)
    , m_turretHomed(false)
    , m_manualOverride(false)
    , m_shooterSpeedMap(shooterRange::shooterSpeed)
    , m_hoodAngleMap(shooterRange::hoodAngle)
    , m_hoodPIDTuner{"argos/hood",
                     {&m_hoodMotor},
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
                       {&m_turretMotor},
                       0,
                       argos_lib::ClosedLoopSensorConversions{
                           argos_lib::GetSensorConversionFactor(sensor_conversions::turret::ToAngle),
                           1.0,
                           argos_lib::GetSensorConversionFactor(sensor_conversions::turret::ToAngle)}}
    , m_instance(instance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::shooter::shooterWheelLeft,
                                         motorConfig::practice_bot::shooter::shooterWheelLeft>(
      m_shooterWheelLeft, 50_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::shooter::shooterWheelRight,
                                         motorConfig::practice_bot::shooter::shooterWheelRight>(
      m_shooterWheelRight, 50_ms, instance);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::shooter::hoodMotor,
                                             motorConfig::practice_bot::shooter::hoodMotor>(
      m_hoodMotor, 50_ms, instance);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::shooter::turretMotor,
                                             motorConfig::practice_bot::shooter::turretMotor>(
      m_turretMotor, 50_ms, instance);

  InitializeTurretHome();

  m_shooterWheelRight.Follow(m_shooterWheelLeft);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber(
      "(Turret) absAngle",
      sensor_conversions::turret::ToAngle(m_turretMotor.GetSensorCollection().GetPulseWidthPosition()).to<double>());
  frc::SmartDashboard::PutNumber(
      "(Turret) relAngle", sensor_conversions::turret::ToAngle(m_turretMotor.GetSelectedSensorPosition()).to<double>());
  const auto turretNormalizedPosition = TurretGetPosition();
  frc::SmartDashboard::PutNumber("(Turret) normalizedAngle",
                                 turretNormalizedPosition ? turretNormalizedPosition.value().to<double>() : NAN);
}

void ShooterSubsystem::AutoAim() {
  // Get target angle & assign to turret
  m_cameraInterface.SetDriverMode(false);
  std::optional<photonlib::PhotonTrackedTarget> hightestTarget = m_cameraInterface.GetHighestTarget();
  frc::SmartDashboard::PutBoolean("Target Exists?", (hightestTarget != std::nullopt));
  if (!hightestTarget) {
    return;
  }
  units::angle::degree_t targetAngle = GetTurretTargetAngle(hightestTarget.value());
  frc::SmartDashboard::PutNumber("Target turret angle", targetAngle.to<double>());
  TurretSetPosition(targetAngle);

  // Get target distance & assign to hood & shooter
  units::length::inch_t distanceToTarget =
      GetTargetDistance(units::make_unit<units::angle::degree_t>(hightestTarget->GetPitch()));
  frc::SmartDashboard::PutNumber("Target distance", distanceToTarget.to<double>());
  SetShooterDistance(distanceToTarget);
}

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
    // Turret is backward on practice bot, but this is simpler than changing homing logic
    MoveTurret(m_instance == argos_lib::RobotInstance::Practice ? -turnSpeed : turnSpeed);
    MoveHood(hoodSpeed);
  }
}

void ShooterSubsystem::MoveHood(double hoodSpeed) {
  m_hoodMotor.Set(hoodSpeed);
}

void ShooterSubsystem::MoveTurret(double turnSpeed) {
  m_turretMotor.Set(turnSpeed);
}

void ShooterSubsystem::HoodSetPosition(units::degree_t angle) {
  if (IsHoodHomed()) {
    m_manualOverride = false;
    angle *= -1;
    m_hoodMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, sensor_conversions::hood::ToSensorUnit(angle));
  }
}

void ShooterSubsystem::UpdateHoodHome() {
  m_hoodMotor.SetSelectedSensorPosition(sensor_conversions::hood::ToSensorUnit(measure_up::hood::homeAngle));
  m_hoodHomed = true;
}

bool ShooterSubsystem::IsHoodMoving() {
  return std::fabs(m_hoodMotor.GetSelectedSensorVelocity()) > 5.0;
}

bool ShooterSubsystem::IsHoodHomed() {
  return m_hoodHomed;
}

bool ShooterSubsystem::IsManualOverride() {
  return m_manualOverride;
}

void ShooterSubsystem::UpdateTurretHome() {
  const auto currentAngle = argos_lib::swerve::ConstrainAngle(
      sensor_conversions::turret::ToAngle(m_turretMotor.GetSensorCollection().GetPulseWidthPosition()), 0_deg, 360_deg);
  m_turretMotor.SetSelectedSensorPosition(
      sensor_conversions::turret::ToSensorUnit(360_deg - measure_up::turret::homeAngle));
  if (!m_turretHomingStorage.Save(currentAngle - measure_up::turret::homeAngle)) {
    std::printf("*************Failed to save************\n");
  }
  m_turretHomed = true;
  SetTurretSoftLimits();
}

void ShooterSubsystem::InitializeTurretHome() {
  const auto homePosition = m_turretHomingStorage.Load();
  if (homePosition) {
    const auto currentAngle = argos_lib::swerve::ConstrainAngle(
        sensor_conversions::turret::ToAngle(m_turretMotor.GetSensorCollection().GetPulseWidthPosition()),
        0_deg,
        360_deg);
    m_turretMotor.SetSelectedSensorPosition(sensor_conversions::turret::ToSensorUnit(
        argos_lib::swerve::ConstrainAngle(currentAngle - homePosition.value(), 0_deg, 360_deg)));
    m_turretHomed = true;
    SetTurretSoftLimits();
  } else {
    DisableTurretSoftLimits();
  }
}

bool ShooterSubsystem::IsTurretHomed() {
  return m_turretHomed;
}

void ShooterSubsystem::TurretSetPosition(units::degree_t angle) {
  if (IsTurretHomed()) {
    m_manualOverride = false;
    m_turretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                      sensor_conversions::turret::ToSensorUnit(
                          m_instance == argos_lib::RobotInstance::Competition ? 360_deg - angle : angle));
  }
}

std::optional<units::degree_t> ShooterSubsystem::TurretGetPosition() {
  if (IsTurretHomed()) {
    units::degree_t rawAngle = sensor_conversions::turret::ToAngle(m_turretMotor.GetSelectedSensorPosition());
    // Competition robot has inverted sensor positions :(
    switch (m_instance) {
      case argos_lib::RobotInstance::Competition:
        return 360_deg - rawAngle;
      case argos_lib::RobotInstance::Practice:
        return rawAngle;
    }
  }
  return std::nullopt;
}

void ShooterSubsystem::SetTurretSoftLimits() {
  units::degree_t minAngle = m_instance == argos_lib::RobotInstance::Competition ?
                                 360_deg - measure_up::turret::maxAngle :
                                 measure_up::turret::minAngle;
  units::degree_t maxAngle = m_instance == argos_lib::RobotInstance::Competition ?
                                 360_deg - measure_up::turret::minAngle :
                                 measure_up::turret::maxAngle;

  m_turretMotor.ConfigForwardSoftLimitThreshold(sensor_conversions::turret::ToSensorUnit(maxAngle));
  m_turretMotor.ConfigReverseSoftLimitThreshold(sensor_conversions::turret::ToSensorUnit(minAngle));
  m_turretMotor.ConfigForwardSoftLimitEnable(true);
  m_turretMotor.ConfigReverseSoftLimitEnable(true);
}

void ShooterSubsystem::DisableTurretSoftLimits() {
  m_turretMotor.ConfigForwardSoftLimitEnable(false);
  m_turretMotor.ConfigReverseSoftLimitEnable(false);
}

void ShooterSubsystem::Disable() {
  m_manualOverride = false;
}

units::inch_t ShooterSubsystem::GetTargetDistance(units::degree_t targetVerticalAngle) {
  return (measure_up::camera::upperHubHeight - measure_up::camera::cameraHeight) /
         std::tan(
             static_cast<units::radian_t>(measure_up::camera::cameraMountAngle + targetVerticalAngle).to<double>());
}

void ShooterSubsystem::SetShooterDistance(units::inch_t distanceToTarget) {
  CloseLoopShoot(units::revolutions_per_minute_t{m_shooterSpeedMap.Map(distanceToTarget.to<double>())});
  HoodSetPosition(units::degree_t{m_hoodAngleMap.Map(distanceToTarget.to<double>())});
}

// CAMERA INTERFACE -----------------------------------------------------------------------------
CameraInterface::CameraInterface() : m_camera{"10.17.56.122"} {
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

void CameraInterface::SetDriverMode(bool mode) {
  m_camera.SetDriverMode(mode);
}

std::optional<units::degree_t> ShooterSubsystem::GetTurretTargetAngle(photonlib::PhotonTrackedTarget target) {
  units::length::inch_t cameraToTargetDistance =
      GetTargetDistance(units::make_unit<units::degree_t>(target.GetPitch()));
  units::length::inch_t cameraTurretOffset = measure_up::camera::toRotationCenter;
  // calculate d2
  double turretToTargetDistance =
      std::sqrt(std::pow(cameraTurretOffset.to<double>(), 2.0) + std::pow(cameraToTargetDistance.to<double>(), 2.0));

  auto currentAngle = TurretGetPosition();
  if (currentAngle) {
    units::degree_t targetAngle = units::make_unit<units::degree_t>(
        currentAngle.value().to<double>() + std::acos((std::pow(cameraTurretOffset.to<double>(), 2.0) +
                                                       turretToTargetDistance - cameraToTargetDistance.to<double>()) /
                                                      2 * cameraTurretOffset.to<double>() * turretToTargetDistance));
    return targetAngle;
  }
  return std::nullopt;
}
