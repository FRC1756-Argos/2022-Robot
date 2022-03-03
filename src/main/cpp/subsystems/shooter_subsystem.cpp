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

  m_cameraInterface.SetDriverMode(true);
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
  LimelightTarget::tValues targetValues = m_cameraInterface.m_target.GetTarget();

  // Get target angle & assign to turret
  m_cameraInterface.SetDriverMode(false);
  if (!m_cameraInterface.m_target.HasTarget()) {
    return;
  }

  frc::SmartDashboard::PutBoolean("(Auto-Aim) Is Highest Target Present?", m_cameraInterface.m_target.HasTarget());
  frc::SmartDashboard::PutNumber("(Auto-Aim) Target Pitch", targetValues.pitch.to<double>());
  frc::SmartDashboard::PutNumber("(Auto-Aim) Target Yaw", targetValues.yaw.to<double>());

  std::optional<units::degree_t> targetAngle = GetTurretTargetAngle(targetValues);
  if (targetAngle) {
    TurretSetPosition(targetAngle.value());
  }

  frc::SmartDashboard::PutNumber("(Auto-Aim) Turret target angle", targetAngle.value().to<double>());
  frc::SmartDashboard::PutNumber(
      "(Turret) relAngle", sensor_conversions::turret::ToAngle(m_turretMotor.GetSelectedSensorPosition()).to<double>());

  // Get target distance & assign to hood & shooter
  units::length::inch_t distanceToTarget = GetTargetDistance(targetValues.pitch);
  SetShooterDistance(distanceToTarget);

  frc::SmartDashboard::PutNumber("(Auto-Aim) Target distance", distanceToTarget.to<double>());
}

void ShooterSubsystem::FixedShooterPosition(FixedPosState fixedPosState) {
  switch (fixedPosState) {
    case FixedPosState::Front:
      TurretSetPosition(measure_up::closepositions::fixedFrontPos);
      SetShooterDistance(measure_up::closepositions::fixedLongDist);
      break;
    case FixedPosState::Left:
      TurretSetPosition(measure_up::closepositions::fixedLeftPos);
      SetShooterDistance(measure_up::closepositions::fixedShortDist);
      break;
    case FixedPosState::Right:
      TurretSetPosition(measure_up::closepositions::fixedRightPos);
      SetShooterDistance(measure_up::closepositions::fixedShortDist);
      break;
    case FixedPosState::Back:
      TurretSetPosition(measure_up::closepositions::fixedBackPos);
      SetShooterDistance(measure_up::closepositions::fixedLongDist);
      break;
  }
}

void ShooterSubsystem::ManualOverride() {
  m_manualOverride = true;
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

units::degree_t ShooterSubsystem::GetHoodPosition() {
  return sensor_conversions::hood::ToAngle(m_hoodMotor.GetSelectedSensorPosition());
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
    // See note in header about angle wrap around
    const auto wrapAroundProtectedAngle = currentAngle < 10_deg ? currentAngle + 360_deg : currentAngle;
    m_turretMotor.SetSelectedSensorPosition(
        sensor_conversions::turret::ToSensorUnit(wrapAroundProtectedAngle - homePosition.value()));
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
  m_cameraInterface.SetDriverMode(true);
  m_shooterWheelLeft.Set(0);
  m_hoodMotor.Set(0);
  m_turretMotor.Set(0);
}

units::inch_t ShooterSubsystem::GetTargetDistance(units::degree_t targetVerticalAngle) {
  return (measure_up::camera::upperHubHeight - measure_up::camera::cameraHeight) /
         std::tan(
             static_cast<units::radian_t>(measure_up::camera::cameraMountAngle + targetVerticalAngle).to<double>());
}

void ShooterSubsystem::SetShooterDistance(units::inch_t distanceToTarget) {
  CloseLoopShoot(units::revolutions_per_minute_t{m_shooterSpeedMap.Map(distanceToTarget)});
  HoodSetPosition(units::degree_t{m_hoodAngleMap.Map(distanceToTarget)});
}

std::optional<units::degree_t> ShooterSubsystem::GetTurretTargetAngle(LimelightTarget::tValues target) {
  units::length::inch_t cameraToTargetDistance = GetTargetDistance(target.pitch);

  units::degree_t alpha{180 - std::abs(target.yaw.to<double>())};

  units::length::inch_t turretToTargetDistance = units::make_unit<units::length::inch_t>(
      std::sqrt(std::pow(cameraToTargetDistance.to<double>(), 2.0) +
                std::pow(measure_up::camera::toRotationCenter.to<double>(), 2.0) -
                2 * measure_up::camera::toRotationCenter.to<double>() * cameraToTargetDistance.to<double>() *
                    std::cos(units::radian_t{alpha}.to<double>())));

  std::optional<units::angle::degree_t> currentTurretAngle = TurretGetPosition();

  units::radian_t offset{
      std::asin((cameraToTargetDistance.to<double>() * std::sin(units::radian_t{alpha}.to<double>())) /
                turretToTargetDistance.to<double>())};

  if (!currentTurretAngle) {
    return std::nullopt;
  }

  units::angle::degree_t targetAngle;

  if (target.yaw.to<double>() >= 0) {
    targetAngle = currentTurretAngle.value() - offset;
  } else {
    targetAngle = currentTurretAngle.value() + offset;
  }

  return targetAngle;
}

LimelightTarget::tValues ShooterSubsystem::GetCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget();
}

// CAMERA INTERFACE -----------------------------------------------------------------------------
CameraInterface::CameraInterface() {}

void CameraInterface::SetDriverMode(bool mode) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  int requestedPipeline = mode ? camera::driverPipeline : camera::targetingPipeline;

  table->PutNumber("pipeline", requestedPipeline);
}

// LIMELIGHT TARGET MEMBER FUNCTIONS ===============================================================

LimelightTarget::tValues LimelightTarget::GetTarget() {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  m_yaw = units::make_unit<units::degree_t>(table->GetNumber("tx", 0.0));
  m_pitch = units::make_unit<units::degree_t>(table->GetNumber("ty", 0.0));
  m_hasTargets = (table->GetNumber("tv", 0) == 1);

  tValues targetValues{m_pitch, m_yaw};
  return targetValues;
}

bool LimelightTarget::HasTarget() {
  return m_hasTargets;
}
