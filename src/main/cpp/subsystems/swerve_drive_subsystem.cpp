/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/swerve_drive_subsystem.h"

#include <argos_lib/config/cancoder_config.h>
#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/swerve_utils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <memory>

#include "Constants.h"
#include "utils/sensor_conversions.h"

using namespace argos_lib::swerve;

SwerveDriveSubsystem::SwerveDriveSubsystem(std::shared_ptr<NetworkTablesWrapper> networkTable,
                                           const argos_lib::RobotInstance instance)
    : m_frontLeft(address::drive::frontLeftDrive, address::drive::frontLeftTurn, address::encoders::frontLeftEncoder)
    , m_frontRight(
          address::drive::frontRightDrive, address::drive::frontRightTurn, address::encoders::frontRightEncoder)
    , m_backRight(address::drive::backRightDrive, address::drive::backRightTurn, address::encoders::backRightEncoder)
    , m_backLeft(address::drive::backLeftDrive, address::drive::backLeftTurn, address::encoders::backLeftEncoder)
    , m_imu(frc::ADIS16448_IMU::kZ, frc::SPI::Port::kMXP, frc::ADIS16448_IMU::CalibrationTime::_4s)
    , m_pNetworkTable(networkTable)
    , m_fsStorage(paths::swerveHomesPath) {
  // create our translation objects

  // TURN MOTORS CONFIG
  std::printf("Configure turn\n");
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::frontLeftTurn,
                                         motorConfig::practice_bot::drive::frontLeftTurn>(
      m_frontLeft.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::frontRightTurn,
                                         motorConfig::practice_bot::drive::frontRightTurn>(
      m_frontRight.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::backRightTurn,
                                         motorConfig::practice_bot::drive::backRightTurn>(
      m_backRight.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::backLeftTurn,
                                         motorConfig::practice_bot::drive::backLeftTurn>(
      m_backLeft.m_turn, 100_ms, instance);

  // DRIVE MOTOR CONFIGS
  std::printf("Configure drive\n");
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_frontLeft.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_frontRight.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_backLeft.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_backRight.m_drive, 100_ms, instance);

  // CAN ENCODER CONFIG
  std::printf("Configure encoders\n");
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::frontLeftTurn,
                                             motorConfig::practice_bot::drive::frontLeftTurn>(
      m_frontLeft.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::frontRightTurn,
                                             motorConfig::practice_bot::drive::frontRightTurn>(
      m_frontRight.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::backRightTurn,
                                             motorConfig::practice_bot::drive::backRightTurn>(
      m_backRight.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::comp_bot::drive::backLeftTurn,
                                             motorConfig::practice_bot::drive::backLeftTurn>(
      m_backLeft.m_encoder, 100_ms, instance);

  // TRANSLATION2D OBJECTS DESCRIBING LOCATION OF SWERVE MODULES
  // Forward is positive X, left is positive Y
  frc::Translation2d frontLeftCenterOffset(
      measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontLeftLOffset,
      measure_up::chassis::width / 2 - measure_up::swerve_offsets::frontLeftWOffset);
  frc::Translation2d frontRightCenterOffset(
      measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontRightLOffset,
      -measure_up::chassis::width / 2 + measure_up::swerve_offsets::frontRightWOffset);
  frc::Translation2d backRightCenterOffset(
      -measure_up::chassis::length / 2 + measure_up::swerve_offsets::backRightLOffset,
      -measure_up::chassis::width / 2 + measure_up::swerve_offsets::backRightWOffset);
  frc::Translation2d backLeftCenterOffset(
      -measure_up::chassis::length / 2 + measure_up::swerve_offsets::backLeftLOffset,
      measure_up::chassis::width / 2 - measure_up::swerve_offsets::backLeftWOffset

  );

  m_controlMode = SwerveDriveSubsystem::DriveControlMode::fieldCentricControl;

  m_pSwerveDriveKinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(
      frontLeftCenterOffset, frontRightCenterOffset, backRightCenterOffset, backLeftCenterOffset);

  InitializeMotors();
}

void SwerveDriveSubsystem::Disable() {
  m_controlMode = DriveControlMode::fieldCentricControl;
  StopDrive();
}

// This method will be called once per scheduler run
void SwerveDriveSubsystem::Periodic() {}

// SWERVE DRIVE SUBSYSTEM MEMBER FUNCTIONS

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetRawModuleStates(
    SwerveDriveSubsystem::Velocities velocities) {
  // IF SPEEDS ZERO, SET MOTORS TO ZERO AND RETURN
  if (velocities.fwVelocity == 0 && velocities.sideVelocity == 0 && velocities.rotVelocity == 0) {
    StopDrive();
    frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

    return m_pSwerveDriveKinematics->ToSwerveModuleStates(emptySpeeds);
  }

  switch (m_controlMode) {
    case (DriveControlMode::
              fieldCentricControl): {  // Construct speeds with field-relative speeds and current IMU Z angle.
      frc::ChassisSpeeds fieldCentricSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          units::make_unit<units::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity),
          frc::Rotation2d(-m_imu.GetAngle() - m_fieldHomeOffset));

      // Return the speeds to consumer
      return m_pSwerveDriveKinematics->ToSwerveModuleStates(fieldCentricSpeeds);
    }

    case (DriveControlMode::robotCentricControl): {
      // Construct speeds just the same as in the current main drive function
      frc::ChassisSpeeds robotCentricSpeeds{
          units::make_unit<units::velocity::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::velocity::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity)};

      return m_pSwerveDriveKinematics->ToSwerveModuleStates(robotCentricSpeeds);
    }
  }
  frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

  return m_pSwerveDriveKinematics->ToSwerveModuleStates(emptySpeeds);
}

void SwerveDriveSubsystem::SwerveDrive(const double& fwVelocity,
                                       const double& sideVelocity,
                                       const double& rotVelocity) {
  if (fwVelocity == 0 && sideVelocity == 0 && rotVelocity == 0) {
    StopDrive();
    return;
  }

  SwerveDriveSubsystem::Velocities velocities{fwVelocity, sideVelocity, rotVelocity};

  // DEBUG STUFF
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) fwVelocity", fwVelocity);
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) sideVelocity", sideVelocity);
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) rotVelocity", rotVelocity);
  frc::SmartDashboard::PutNumber("CONTROL MODE", m_controlMode);
  frc::SmartDashboard::PutNumber("IMU ANGLE", m_imu.GetAngle().to<double>());

  // SET MODULES BASED OFF OF CONTROL MODE
  auto moduleStates = GetRawModuleStates(velocities);

  moduleStates.at(0) = argos_lib::swerve::Optimize(
      moduleStates.at(0),
      sensor_conversions::swerve_drive::turn::ToAngle(m_frontLeft.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);
  moduleStates.at(1) = argos_lib::swerve::Optimize(
      moduleStates.at(1),
      sensor_conversions::swerve_drive::turn::ToAngle(m_frontRight.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);
  moduleStates.at(2) = argos_lib::swerve::Optimize(
      moduleStates.at(2),
      sensor_conversions::swerve_drive::turn::ToAngle(m_backRight.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);
  moduleStates.at(3) = argos_lib::swerve::Optimize(
      moduleStates.at(3),
      sensor_conversions::swerve_drive::turn::ToAngle(m_backLeft.m_turn.GetSelectedSensorPosition()),
      0_rpm,
      0_fps,
      12_fps);

  // Give module state values to motors

  // FRONT LEFT
  m_frontLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                          moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>());

  m_frontLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                         sensor_conversions::swerve_drive::turn::ToSensorUnit(
                             moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees()));

  // FRONT RIGHT
  m_frontRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                           moduleStates.at(indexes::swerveModules::frontRightIndex).speed.to<double>());

  m_frontRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          sensor_conversions::swerve_drive::turn::ToSensorUnit(
                              moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees()));

  // BACK RIGHT
  m_backRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                          moduleStates.at(indexes::swerveModules::backRightIndex).speed.to<double>());

  m_backRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                         sensor_conversions::swerve_drive::turn::ToSensorUnit(
                             moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees()));

  // BACK LEFT
  m_backLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
                         moduleStates.at(indexes::swerveModules::backLeftIndex).speed.to<double>());

  m_backLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                        sensor_conversions::swerve_drive::turn::ToSensorUnit(
                            moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees()));

  // DEBUG STUFF
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FL speed",
                                 moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FL turn",
                                 moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees().to<double>());

  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FR speed",
                                 moduleStates.at(indexes::swerveModules::frontRightIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) FR turn",
                                 moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees().to<double>());

  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BR speed",
                                 moduleStates.at(indexes::swerveModules::backRightIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BR turn",
                                 moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees().to<double>());

  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BL speed",
                                 moduleStates.at(indexes::swerveModules::backLeftIndex).speed.to<double>());
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) BL turn",
                                 moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees().to<double>());
}

void SwerveDriveSubsystem::StopDrive() {
  m_frontLeft.m_drive.Set(0.0);
  m_frontLeft.m_turn.Set(0.0);
  m_frontRight.m_drive.Set(0.0);
  m_frontRight.m_turn.Set(0.0);
  m_backRight.m_drive.Set(0.0);
  m_backRight.m_turn.Set(0.0);
  m_backLeft.m_drive.Set(0.0);
  m_backLeft.m_turn.Set(0.0);
}

void SwerveDriveSubsystem::Home(const units::degree_t& angle) {
  HomeToFS(angle);

  // RE-ZERO THE IMU
  m_imu.Reset();

  // SetPosition expects a value in degrees
  m_frontLeft.m_encoder.SetPosition(angle.to<double>(), 50);
  m_frontRight.m_encoder.SetPosition(angle.to<double>(), 50);
  m_backRight.m_encoder.SetPosition(angle.to<double>(), 50);
  m_backLeft.m_encoder.SetPosition(angle.to<double>(), 50);
}

void SwerveDriveSubsystem::FieldHome(units::degree_t homeAngle) {
  m_fieldHomeOffset = -m_imu.GetAngle() - homeAngle;
}

void SwerveDriveSubsystem::SetControlMode(SwerveDriveSubsystem::DriveControlMode controlMode) {
  m_controlMode = controlMode;
}

void SwerveDriveSubsystem::InitializeMotors() {
  InitializeMotorsFromFS();
}

void SwerveDriveSubsystem::HomeToFS(const units::degree_t& angle) {
  const argos_lib::swerve::SwerveModulePositions homes{
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg)};

  m_fsStorage.Save(homes);
}

void SwerveDriveSubsystem::InitializeMotorsFromFS() {
  std::optional<argos_lib::swerve::SwerveModulePositions> homes = m_fsStorage.Load();

  if (!homes) {
    // ALERT HERE THAT THERE ARE NO VALUES, BUT FOR NOW, JUST PRINT
    std::printf("%d HEY NO SAVED VALUES IN std::FILE SYSTEM!!!!", __LINE__);
    return;
  }

  // GET CURRENT VALUES
  units::degree_t frontLeft_current = units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition());
  units::degree_t frontRight_current = units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition());
  units::degree_t backRight_current = units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition());
  units::degree_t backLeft_current = units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition());

  // SUBTRACT SAVED FROM CURRENT
  const units::degree_t frontLeftCalibrated = frontLeft_current - homes.value().FrontLeft;
  const units::degree_t frontRightCalibrated = frontRight_current - homes.value().FrontRight;
  const units::degree_t backRightCalibrated = backRight_current - homes.value().RearRight;
  const units::degree_t backLeftCalibrated = backLeft_current - homes.value().RearLeft;

  // ASSIGN DIFFERENCE TO CURRENT MOTOR RELATIVE POSITION
  m_frontLeft.m_encoder.SetPosition(frontLeftCalibrated.to<double>(), 50);
  m_frontRight.m_encoder.SetPosition(frontRightCalibrated.to<double>(), 50);
  m_backRight.m_encoder.SetPosition(backRightCalibrated.to<double>(), 50);
  m_backLeft.m_encoder.SetPosition(backLeftCalibrated.to<double>(), 50);
}

// SWERVE MODULE SUBSYSTEM FUNCTIONS
SwerveModule::SwerveModule(const char driveAddr, const char turnAddr, const char encoderAddr)

    : m_drive(driveAddr), m_turn(turnAddr), m_encoder(encoderAddr) {}
