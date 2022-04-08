/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/shooter_subsystem.h"

#include <frc/drive/Vector2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"
#include "argos_lib/config/talonsrx_config.h"
#include "argos_lib/general/swerve_utils.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/length.h"
#include "utils/general.h"
#include "utils/sensor_conversions.h"

ShooterSubsystem::ShooterSubsystem(const argos_lib::RobotInstance instance,
                                   SwerveDriveSubsystem* pDriveSubsystem,
                                   argos_lib::SwappableControllersSubsystem* controllers)
    : m_shooterWheelLeft(address::shooter::shooterWheelLeft)
    , m_shooterWheelRight(address::shooter::shooterWheelRight)
    , m_hoodMotor(address::shooter::hoodMotor)
    , m_turretMotor(address::shooter::turretMotor)
    , m_cameraInterface()
    , m_turretHomingStorage(paths::turretHomePath)
    , m_hoodHomed(false)
    , m_turretHomed(false)
    , m_manualOverride(false)
    , m_useCalculatedPitch(true)
    , m_shooterSpeedMap(shooterRange::shooterSpeed)
    , m_hoodAngleMap(shooterRange::hoodAngle)
    , m_lateralSpeedMap(shooterRange::lateralSpeed)
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
    , m_instance(instance)
    , m_pDriveSubsystem(pDriveSubsystem)
    , m_pControllers(controllers) {
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

  if (m_turretMotor.HasResetOccurred()) {
    m_turretMotor.Set(0);
    std::printf("***Turret reset!  Re-initializing home***\n");
    m_turretHomed = false;
    InitializeTurretHome();
  }
}

bool ShooterSubsystem::AutoAim(bool drivingAdjustment) {
  LimelightTarget::tValues targetValues = m_cameraInterface.m_target.GetTarget();

  // Get target angle & assign to turret
  m_cameraInterface.SetDriverMode(false);
  if (!m_cameraInterface.m_target.HasTarget()) {
    StopFeedback();
    return false;
  }

  frc::SmartDashboard::PutBoolean("(Auto-Aim) Is Highest Target Present?", m_cameraInterface.m_target.HasTarget());
  frc::SmartDashboard::PutNumber("(Auto-Aim) Target Pitch", targetValues.pitch.to<double>());
  frc::SmartDashboard::PutNumber("(Auto-Aim) Target Yaw", targetValues.yaw.to<double>());

  std::optional<units::degree_t> targetAngle = GetTurretTargetAngle(targetValues);

  frc::SmartDashboard::PutNumber("(Auto-Aim) Turret target angle", targetAngle.value().to<double>());
  const auto currentTurretAngle = sensor_conversions::turret::ToAngle(m_turretMotor.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("(Turret) relAngle", currentTurretAngle.to<double>());

  // Get target distance & assign to hood & shooter
  units::length::inch_t distanceToTarget;

  units::length::inch_t fudgeFactor = 12_in;  // 12_in at CIR and now comp bot on practice field?

  if (m_useCalculatedPitch) {
    units::degree_t newPitch = m_cameraInterface.GetNewPitch(
        targetValues.yaw, targetValues.pitch, targetValues.bboxHor, targetValues.bboxVer, targetValues.skew);
    frc::SmartDashboard::PutNumber("(Auto-Aim) Calculated Pitch", newPitch.to<double>());
    m_cameraInterface.m_target.adjustPerspective(newPitch, targetValues.yaw);
    distanceToTarget = GetTargetDistance(newPitch);
    frc::SmartDashboard::PutNumber("(Auto-Aim) Perspective Adjusted Pitch", newPitch.to<double>());
  } else {
    m_cameraInterface.m_target.adjustPerspective(targetValues.pitch, targetValues.yaw);
    frc::SmartDashboard::PutNumber("(Auto-Aim) Perspective Adjusted Pitch", targetValues.pitch.to<double>());
    distanceToTarget = GetTargetDistance(targetValues.pitch);
  }

  frc::SmartDashboard::PutNumber("(Auto-Aim) Target distance without offset", distanceToTarget.to<double>());

  // fitting 2nd degree polynomial to get the offset
  distanceToTarget -= GetPolynomialOffset(distanceToTarget);

  // if (distanceToTarget >= 160_in) {
  //   distanceToTarget -= 2_in;
  // } else {
  //   distanceToTarget += 2_in;
  // }

  distanceToTarget += fudgeFactor;

  frc::SmartDashboard::PutNumber("(Auto-Aim) Target distance before motion adjust", distanceToTarget.to<double>());

  if (drivingAdjustment) {
    const auto drivingAdjustmentValues = DrivingAimOffsets(
        ChassisVelocitiesToHubVelocities(m_pDriveSubsystem->GetChassisVelocity(), targetAngle.value()),
        distanceToTarget,
        targetAngle.value(),
        targetValues.totalLatency);
    // Raw adjustments were too powerful...
    distanceToTarget += (drivingAdjustmentValues.distanceOffset * 1.0);
    targetAngle.value() -=
        (drivingAdjustmentValues.yawOffset * 0.8) * (m_instance == argos_lib::RobotInstance::Competition ? -1.0 : 1.0);
  }

  frc::SmartDashboard::PutNumber("(Auto-Aim) Target distance final", distanceToTarget.to<double>());

  const auto shooterSetpoints = SetShooterDistance(distanceToTarget);

  const AimValues targets{
      (targetAngle ? targetAngle.value() : 0_deg), shooterSetpoints.hoodAngle, shooterSetpoints.shooterSpeed};
  // Note, hood reads negative angles, so have to negage normalized position for range check
  const AimValues currentValues{
      m_instance == argos_lib::RobotInstance::Competition ? (360_deg - currentTurretAngle) : currentTurretAngle,
      GetHoodPosition() * -1,
      GetShooterSpeed()};

  if (targetAngle) {
    TurretSetPosition(targetAngle.value());
  }

  if (InAcceptableRanges(targets, currentValues)) {
    AimedFeedback();
    return true;
  } else {
    StopFeedback();
    return false;
  }
}

units::inch_t ShooterSubsystem::GetPolynomialOffset(units::inch_t actualDistance) {
  units::inch_t offset = 0_in;
  double camDegOffsetAcounting;
  if (m_instance == argos_lib::RobotInstance::Competition) {
    camDegOffsetAcounting = 2.928571;
  } else {
    camDegOffsetAcounting = -17.071429;
  }

  if (actualDistance >= (units::inch_t)160) {
    if (m_instance == argos_lib::RobotInstance::Competition) {
      double y =
          (50 - (0.4166667 * actualDistance.to<double>()) + (0.001388889 * std::pow(actualDistance.to<double>(), 2)));
      offset = (units::inch_t)y;
    } else {
      double y = (8 - (0.1 * actualDistance.to<double>()) - (0.00028 * std::pow(actualDistance.to<double>(), 2)));
      offset = (units::inch_t)y;
    }
  } else if (actualDistance >= (units::inch_t)90 && actualDistance < (units::inch_t)160) {
    if (m_instance == argos_lib::RobotInstance::Competition) {
      double y = (camDegOffsetAcounting - (0.1528571 * actualDistance.to<double>()) +
                  (0.00148571 * std::pow(actualDistance.to<double>(), 2)));
      offset = units::inch_t{y};
    } else {
      offset = -15_in;
    }
  } else if (m_instance == argos_lib::RobotInstance::Practice) {
    offset = -18_in;  // for practice bot, needs further tuning!!!
  }
  return offset;
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

units::angular_velocity::revolutions_per_minute_t ShooterSubsystem::GetShooterSpeed() {
  return units::angular_velocity::revolutions_per_minute_t{m_shooterWheelLeft.GetSelectedSensorVelocity() *
                                                           sensor_conversions::shooter::sensorConversionFactor};
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
  m_hoodMotor.SetSelectedSensorPosition(sensor_conversions::hood::ToSensorUnit(
      m_instance == argos_lib::RobotInstance::Competition ? measure_up::hood::comp_bot::homeAngle :
                                                            measure_up::hood::practice_bot::homeAngle));
  SetHoodSoftLimits();
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

void ShooterSubsystem::SetHoodSoftLimits() {
  units::degree_t maxAngle = m_hoodAngleMap(0_ft) * -1;
  units::degree_t minAngle = m_hoodAngleMap(35_ft) * -1;

  frc::SmartDashboard::PutNumber("(Hood) Forward Soft Limit", maxAngle.to<double>());
  frc::SmartDashboard::PutNumber("(Hood) Reverse Soft Limit", minAngle.to<double>());

  m_hoodMotor.ConfigForwardSoftLimitThreshold(sensor_conversions::hood::ToSensorUnit(maxAngle));
  m_hoodMotor.ConfigReverseSoftLimitThreshold(sensor_conversions::hood::ToSensorUnit(minAngle));
  m_hoodMotor.ConfigForwardSoftLimitEnable(true);
  m_hoodMotor.ConfigReverseSoftLimitEnable(true);
}

void ShooterSubsystem::DisableHoodSoftLimits() {
  frc::SmartDashboard::PutNumber("(Hood) Forward Soft Limit", NAN);
  frc::SmartDashboard::PutNumber("(Hood) Reverse Soft Limit", NAN);

  m_hoodMotor.ConfigForwardSoftLimitEnable(false);
  m_hoodMotor.ConfigReverseSoftLimitEnable(false);
}

void ShooterSubsystem::Disable() {
  m_manualOverride = false;
  m_cameraInterface.SetDriverMode(true);
  m_shooterWheelLeft.Set(0);
  m_hoodMotor.Set(0);
  m_turretMotor.Set(0);
  StopFeedback();
}

units::inch_t ShooterSubsystem::GetTargetDistance(units::degree_t targetVerticalAngle) {
  units::degree_t mountAngle;

  if (m_instance == argos_lib::RobotInstance::Competition) {
    mountAngle = measure_up::camera::cameraMountAngle;
  } else {
    mountAngle = measure_up::camera::cameraMountAnglePracticeBot;
  }
  return (measure_up::camera::upperHubHeight - measure_up::camera::cameraHeight) /
         std::tan(static_cast<units::radian_t>(mountAngle + targetVerticalAngle).to<double>());
}

ShooterSubsystem::ShooterDistanceSetpoints ShooterSubsystem::GetShooterDistanceSetpoints(
    units::inch_t distanceToTarget) const {
  return ShooterSubsystem::ShooterDistanceSetpoints{m_shooterSpeedMap.Map(distanceToTarget),
                                                    m_hoodAngleMap.Map(distanceToTarget)};
}

ShooterSubsystem::ShooterDistanceSetpoints ShooterSubsystem::SetShooterDistance(units::inch_t distanceToTarget) {
  auto setpoints = GetShooterDistanceSetpoints(distanceToTarget);
  CloseLoopShoot(setpoints.shooterSpeed);
  HoodSetPosition(setpoints.hoodAngle);
  return setpoints;
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

void ShooterSubsystem::SetCameraDriverMode(bool driverMode) {
  m_cameraInterface.SetDriverMode(driverMode);
  if (driverMode) {
    StopFeedback();
  }
}

bool ShooterSubsystem::InAcceptableRanges(const AimValues targets, const AimValues real) {
  const bool turretAcceptableRange =
      InThreshold(real.turretTarget, targets.turretTarget, threshholds::shooter::acceptableTurretError);
  const bool hoodAcceptableRange =
      InThreshold(real.hoodTarget, targets.hoodTarget, threshholds::shooter::acceptableHoodError);
  const bool shooterAcceptableRange =
      InThreshold(real.shooterTarget, targets.shooterTarget, threshholds::shooter::acceptableWheelError);

  frc::SmartDashboard::PutNumber("(Acceptable Error) Target - Turret", targets.turretTarget.to<double>());
  frc::SmartDashboard::PutNumber("(Acceptable Error) Target - Hood", targets.hoodTarget.to<double>());
  frc::SmartDashboard::PutNumber("(Acceptable Error) Target - Shooter", targets.shooterTarget.to<double>());

  frc::SmartDashboard::PutNumber("(Acceptable Error) Current - Turret", real.turretTarget.to<double>());
  frc::SmartDashboard::PutNumber("(Acceptable Error) Current - Hood", real.hoodTarget.to<double>());
  frc::SmartDashboard::PutNumber("(Acceptable Error) Current - Shooter", real.shooterTarget.to<double>());

  frc::SmartDashboard::PutNumber("(Acceptable Error) Tolerance - Turret",
                                 threshholds::shooter::acceptableTurretError.to<double>());
  frc::SmartDashboard::PutNumber("(Acceptable Error) Tolerance - Hood",
                                 threshholds::shooter::acceptableHoodError.to<double>());
  frc::SmartDashboard::PutNumber("(Acceptable Error) Tolerance - Shooter",
                                 threshholds::shooter::acceptableWheelError.to<double>());

  frc::SmartDashboard::PutBoolean("(Acceptable Error) Turret", turretAcceptableRange);
  frc::SmartDashboard::PutBoolean("(Acceptable Error) Hood", hoodAcceptableRange);
  frc::SmartDashboard::PutBoolean("(Acceptable Error) Wheel Speed", shooterAcceptableRange);

  return turretAcceptableRange && hoodAcceptableRange && shooterAcceptableRange;
}

void ShooterSubsystem::StopFeedback() const {
  if (m_pControllers) {
    m_pControllers->DriverController().SetVibration(argos_lib::VibrationOff());
    m_pControllers->OperatorController().SetVibration(argos_lib::VibrationOff());
  }
}

void ShooterSubsystem::AimedFeedback() const {
  if (m_pControllers) {
    m_pControllers->DriverController().SetVibration(argos_lib::VibrationAlternatePulse(500_ms, 1.0));
    m_pControllers->OperatorController().SetVibration(argos_lib::VibrationAlternatePulse(500_ms, 1.0));
  }
}

ShooterSubsystem::HubRelativeVelocities ShooterSubsystem::ChassisVelocitiesToHubVelocities(
    const frc::ChassisSpeeds robotChassisSpeed, units::degree_t hubTurretAngle) {
  ShooterSubsystem::HubRelativeVelocities retVal;

  // Yaw rate is same in both reference frames
  retVal.chassisYawRate = robotChassisSpeed.omega;

  retVal.radialVelocity =
      robotChassisSpeed.vy * units::math::sin(hubTurretAngle) - robotChassisSpeed.vx * units::math::cos(hubTurretAngle);
  retVal.tangentialVelocity =
      robotChassisSpeed.vy * units::math::cos(hubTurretAngle) + robotChassisSpeed.vx * units::math::sin(hubTurretAngle);

  frc::SmartDashboard::PutNumber("(HubRelativeVelocities) v_x",
                                 units::feet_per_second_t{robotChassisSpeed.vx}.to<double>());
  frc::SmartDashboard::PutNumber("(HubRelativeVelocities) v_y",
                                 units::feet_per_second_t{robotChassisSpeed.vy}.to<double>());
  frc::SmartDashboard::PutNumber("(HubRelativeVelocities) omega",
                                 units::degrees_per_second_t{robotChassisSpeed.omega}.to<double>());
  frc::SmartDashboard::PutNumber("(HubRelativeVelocities) hubAngle", units::degree_t{hubTurretAngle}.to<double>());

  frc::SmartDashboard::PutNumber("(HubRelativeVelocities) v_radial",
                                 units::feet_per_second_t{retVal.radialVelocity}.to<double>());
  frc::SmartDashboard::PutNumber("(HubRelativeVelocities) v_tangential",
                                 units::feet_per_second_t{retVal.tangentialVelocity}.to<double>());
  frc::SmartDashboard::PutNumber("(HubRelativeVelocities) yawRate",
                                 units::degrees_per_second_t{retVal.chassisYawRate}.to<double>());

  return retVal;
}

ShooterSubsystem::AimOffsets ShooterSubsystem::DrivingAimOffsets(
    const ShooterSubsystem::HubRelativeVelocities robotVelocity,
    units::foot_t hubDistance,
    units::degree_t hubAngle,
    units::second_t targetStaleness) {
  frc::SmartDashboard::PutNumber("(Driving Shot) hubDistance", hubDistance.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) hubAngle", hubAngle.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) targetStaleness", targetStaleness.to<double>());
  // Motion since last target acquisition
  const units::foot_t radialMotionSinceSample = robotVelocity.radialVelocity * targetStaleness;
  const units::foot_t tangentialMotionSinceSample = robotVelocity.tangentialVelocity * targetStaleness;
  const units::degree_t rotationSinceSample = robotVelocity.chassisYawRate * targetStaleness;

  frc::SmartDashboard::PutNumber("(Driving Shot) radialMotionSinceSample", radialMotionSinceSample.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) tangentialMotionSinceSample",
                                 tangentialMotionSinceSample.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) rotationSinceSample", rotationSinceSample.to<double>());

  const units::foot_t netMotionSinceSample = units::math::sqrt(units::math::pow<2>(radialMotionSinceSample) +
                                                               units::math::pow<2>(tangentialMotionSinceSample));
  const units::degree_t motionAngle = units::math::atan2(radialMotionSinceSample, tangentialMotionSinceSample);

  frc::SmartDashboard::PutNumber("(Driving Shot) netMotionSinceSample", netMotionSinceSample.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) motionAngle", motionAngle.to<double>());

  // Use SAS law of cosines to get hub positions at T=0 (now)
  const units::foot_t hubDistanceT0 =
      units::math::sqrt(units::math::pow<2>(hubDistance) + units::math::pow<2>(netMotionSinceSample) -
                        2 * hubDistance * netMotionSinceSample * units::math::cos(90_deg + motionAngle));
  const units::degree_t hubAngleT0 =
      hubAngle - rotationSinceSample -
      units::math::asin((netMotionSinceSample / hubDistanceT0) * units::math::sin(90_deg + motionAngle));

  frc::SmartDashboard::PutNumber("(Driving Shot) std::asin(hubDistance/hubDistanceT0)", hubDistanceT0.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) hubDistanceT0", hubDistanceT0.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) hubAngleT0", hubAngleT0.to<double>());

  // Motion while ball in transit
  const units::second_t ballTravelTime = hubDistanceT0 / m_lateralSpeedMap(hubDistanceT0);
  const units::foot_t radialMotionInFlight = ballTravelTime * robotVelocity.radialVelocity;
  const units::foot_t tangentialMotionInFlight = ballTravelTime * robotVelocity.tangentialVelocity;
  // Rotation doesn't matter while ball is in flight probably?  Maybe this is a bad assumption, but we'll see

  frc::SmartDashboard::PutNumber("(Driving Shot) ballTravelTime", ballTravelTime.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) radialMotionInFlight", radialMotionInFlight.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) tangentialMotionInFlight", tangentialMotionInFlight.to<double>());

  // Now the fun part... How will the ball move due to initial velocity vectors while in flight?
  const units::foot_t inFlightDistanceAdjustment = radialMotionInFlight;
  const units::degree_t inFlightAngleAdjustment = -units::math::atan2(tangentialMotionInFlight, hubDistanceT0);

  frc::SmartDashboard::PutNumber("(Driving Shot) inFlightDistanceAdjustment", inFlightDistanceAdjustment.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) inFlightAngleAdjustment", inFlightAngleAdjustment.to<double>());

  const AimOffsets offsets{hubDistanceT0 - hubDistanceT0 + inFlightDistanceAdjustment,
                           hubAngleT0 - hubAngle + inFlightAngleAdjustment};

  frc::SmartDashboard::PutNumber("(Driving Shot) offsetDistance", offsets.distanceOffset.to<double>());
  frc::SmartDashboard::PutNumber("(Driving Shot) offsetAngle", offsets.yawOffset.to<double>());

  return offsets;
}

// CAMERA INTERFACE -----------------------------------------------------------------------------
CameraInterface::CameraInterface() {}

void CameraInterface::SetDriverMode(bool mode) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  int requestedPipeline = mode ? camera::driverPipeline : camera::targetingPipeline;

  table->PutNumber("pipeline", requestedPipeline);
}

units::angle::degree_t CameraInterface::HorizontalPixelToAngle(int pixels) {
  return (camera::horizontalAngleResolution * pixels) / camera::horizontalPixelResolution;
}

units::angle::degree_t CameraInterface::VerticalPixelToAngle(int pixels) {
  return (camera::verticalAngleResolution * pixels) / camera::verticalPixelResolution;
}

units::angle::degree_t CameraInterface::GetNewPitch(
    units::degree_t cx, units::degree_t cy, int bboxHorizontalPixels, int bboxVerticalPixels, units::degree_t skew) {
  units::degree_t topLeftCornerX = cx - (HorizontalPixelToAngle(bboxHorizontalPixels) * 0.5);
  units::degree_t topLeftCornerY = cy + (VerticalPixelToAngle(bboxVerticalPixels) * 0.5);
  units::degree_t topRightCornerX = cx + (HorizontalPixelToAngle(bboxHorizontalPixels) * 0.5);
  units::degree_t topRightCornerY = cy + (VerticalPixelToAngle(bboxVerticalPixels) * 0.5);

  // translate
  topLeftCornerX -= cx;
  topLeftCornerY -= cy;
  topRightCornerX -= cx;
  topRightCornerY -= cy;

  // rotate
  frc::Vector2d topLeftCorner(topLeftCornerX.to<double>(), topLeftCornerY.to<double>());
  frc::Vector2d topRightCorner(topRightCornerX.to<double>(), topRightCornerY.to<double>());
  topLeftCorner.Rotate(skew.to<double>());
  topRightCorner.Rotate(skew.to<double>());

  // retranslate
  topLeftCorner.x += cx.to<double>();
  topLeftCorner.y += cy.to<double>();
  topRightCorner.x += cx.to<double>();
  topRightCorner.y += cy.to<double>();

  return units::degree_t{(topLeftCorner.y + topRightCorner.y) * 0.5};
}
// LIMELIGHT TARGET MEMBER FUNCTIONS ===============================================================

LimelightTarget::tValues LimelightTarget::GetTarget() {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  m_yaw = units::make_unit<units::degree_t>(table->GetNumber("tx", 0.0));
  m_pitch = units::make_unit<units::degree_t>(table->GetNumber("ty", 0.0));
  m_bboxHor = (table->GetNumber("tlong", 0.0));
  m_bboxVer = (table->GetNumber("tshort", 0.0));
  m_skew = units::degree_t{(table->GetNumber("ts", 0.0))};
  m_hasTargets = (table->GetNumber("tv", 0) == 1);
  m_pipelineLatency = units::millisecond_t{table->GetNumber("tl", 0.0)};

  tValues targetValues{m_pitch, m_yaw, m_bboxHor, m_bboxVer, m_skew, m_pipelineLatency + m_miscLatency};
  return targetValues;
}

bool LimelightTarget::HasTarget() {
  return m_hasTargets;
}

void LimelightTarget::adjustPerspective(units::degree_t& currentPitch, const units::degree_t& currentYaw) {
  //units::degree_t pitchFactor = 0.001 * currentPitch;
  //units::degree_t perspectiveOffset = pitchFactor * (std::pow(currentYaw.to<double>(), 2));
  //currentPitch -= perspectiveOffset;
  const double yaw_2 = std::pow(currentYaw.to<double>(), 2);
  currentPitch = units::degree_t{(currentPitch.to<double>() * 16000 - yaw_2 * 60) / (16000 + (yaw_2 * 3))};
}
