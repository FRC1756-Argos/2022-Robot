/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/swerve_drive_subsystem.h"

#include <argos_lib/config/cancoder_config.h>
#include <argos_lib/config/falcon_config.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <memory>

#include "Constants.h"

using namespace argos_lib::swerve;

SwerveDriveSubsystem::SwerveDriveSubsystem(std::shared_ptr<network_tables> networkTable)
    : m_frontLeft(address::drive::frontLeftDrive, address::drive::frontLeftTurn, address::encoders::frontLeftEncoder)
    , m_frontRight(
          address::drive::frontRightDrive, address::drive::frontRightTurn, address::encoders::frontRightEncoder)
    , m_backRight(address::drive::backRightDrive, address::drive::backRightTurn, address::encoders::backRightEncoder)
    , m_backLeft(address::drive::backLeftDrive, address::drive::backLeftTurn, address::encoders::backLeftEncoder) {
  // create our translation objects

  // TURN MOTORS CONFIG
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::frontLeftTurn>(m_frontLeft.m_turn, 100_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::frontRightTurn>(m_frontRight.m_turn, 100_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::backRightTurn>(m_backRight.m_turn, 100_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::backLeftTurn>(m_backLeft.m_turn, 100_ms);

  // DRIVE MOTOR CONFIGS
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::genericDrive>(m_frontLeft.m_drive, 100_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::genericDrive>(m_frontRight.m_drive, 100_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::genericDrive>(m_backLeft.m_drive, 100_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::drive::genericDrive>(m_backRight.m_drive, 100_ms);

  // CAN ENCODER CONFIG
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::drive::frontLeftTurn>(m_frontLeft.m_encoder, 100_ms);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::drive::frontRightTurn>(m_frontRight.m_encoder, 100_ms);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::drive::backRightTurn>(m_backRight.m_encoder, 100_ms);
  argos_lib::cancoder_config::CanCoderConfig<motorConfig::drive::backLeftTurn>(m_backLeft.m_encoder, 100_ms);

  // TRANSLATION2D OBJECTS DESCRIBING LOCATION OF SWERVE MODULES
  frc::Translation2d frontLeftCenterOffset(
      -1 * measure_up::chassis::width / 2 + measure_up::swerve_offsets::frontLeftWOffset,
      measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontLeftLOffset);
  frc::Translation2d frontRightCenterOffset(
      measure_up::chassis::width / 2 - measure_up::swerve_offsets::frontRightWOffset,
      measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontRightLOffset);
  frc::Translation2d backRightCenterOffset(
      measure_up::chassis::width / 2 - measure_up::swerve_offsets::backRightWOffset,
      -measure_up::chassis::length / 2 + measure_up::swerve_offsets::backRightLOffset);
  frc::Translation2d backLeftCenterOffset(-measure_up::chassis::width / 2 + measure_up::swerve_offsets::backLeftWOffset,
                                          measure_up::chassis::width / 2 - measure_up::swerve_offsets::backLeftLOffset

  );

  m_pSwerveDriveKinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(
      frontLeftCenterOffset, frontRightCenterOffset, backRightCenterOffset, backLeftCenterOffset);
}

// This method will be called once per scheduler run
void SwerveDriveSubsystem::Periodic() {}

// SEWERVE DRIVE SUBSYSTEM MEMBER FUNCTIONS

void SwerveDriveSubsystem::SwerveDrive(const double& fwVelocity,
                                       const double& sideVelocity,
                                       const double& rotVelocity) {
  frc::ChassisSpeeds speeds{units::make_unit<units::velocity::meters_per_second_t>(fwVelocity),
                            units::make_unit<units::velocity::meters_per_second_t>(sideVelocity),
                            units::make_unit<units::angular_velocity::radians_per_second_t>(rotVelocity)};

  // IF SPEEDS ZERO, SET MOTORS TO ZERO AND RETURN
  if (fwVelocity == 0 && sideVelocity == 0 && rotVelocity == 0) {
    m_frontRight.m_drive.Set(0);
    m_frontRight.m_turn.Set(0);

    m_frontLeft.m_drive.Set(0);
    m_frontLeft.m_turn.Set(0);

    m_backRight.m_drive.Set(0);
    m_backRight.m_turn.Set(0);

    m_backLeft.m_drive.Set(0);
    m_backLeft.m_turn.Set(0);
    return;
  }

  auto moduleStates = m_pSwerveDriveKinematics->ToSwerveModuleStates(speeds);

  /// @todo switch to argosLib optimize functions in time (create overload for meters per second?)
  moduleStates.at(0).Optimize(moduleStates.at(0),
                              units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition()));
  moduleStates.at(1).Optimize(moduleStates.at(1),
                              units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition()));
  moduleStates.at(2).Optimize(moduleStates.at(2),
                              units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition()));
  moduleStates.at(3).Optimize(moduleStates.at(3),
                              units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition()));

  // Give module state values to motors

  // FROMT LEFT
  m_frontLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                          moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>());

  m_frontLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                         moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees().to<double>());

  // FRONT RIGHT
  m_frontRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                           moduleStates.at(indexes::swerveModules::frontRightIndex).speed.to<double>());

  m_frontRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees().to<double>());

  // BACK RIGHT
  m_backRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                          moduleStates.at(indexes::swerveModules::backRightIndex).speed.to<double>());

  m_backRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                         moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees().to<double>());

  // BACK LEFT
  m_backLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                         moduleStates.at(indexes::swerveModules::backLeftIndex).speed.to<double>());

  m_backLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                        moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees().to<double>());
}

void SwerveDriveSubsystem::Home(const units::degree_t& angle) {
  // GET CURRENT HOME POSITION AND SAVE IT
  const SwerveModulePositions homes{
      // GET OUR ABSOLUTE POSITION AND SET IT TO HOME (0 - 360)
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg),
      ConstrainAngle(
          units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition()) - angle, 0_deg, 360_deg)};

  /// @todo switch to argoslib struct save
  m_NetworkTable->m_flHome.SetDouble(homes.FrontLeft.to<double>());
  m_NetworkTable->m_frHome.SetDouble(homes.FrontRight.to<double>());
  m_NetworkTable->m_brHome.SetDouble(homes.RearRight.to<double>());
  m_NetworkTable->m_blHome.SetDouble(homes.RearLeft.to<double>());
}

void SwerveDriveSubsystem::InitializeMotors() {
  // GET SAVED VALUES
  units::degree_t frontLeft_saved = units::make_unit<units::degree_t>(m_NetworkTable->m_flHome.GetDouble(NAN));
  units::degree_t frontRight_saved = units::make_unit<units::degree_t>(m_NetworkTable->m_frHome.GetDouble(NAN));
  units::degree_t backRight_saved = units::make_unit<units::degree_t>(m_NetworkTable->m_brHome.GetDouble(NAN));
  units::degree_t backLeft_saved = units::make_unit<units::degree_t>(m_NetworkTable->m_blHome.GetDouble(NAN));

  if (m_NetworkTable->m_flHome.GetDouble(NAN) == NAN || m_NetworkTable->m_frHome.GetDouble(NAN) == NAN ||
      m_NetworkTable->m_brHome.GetDouble(NAN) == NAN || m_NetworkTable->m_blHome.GetDouble(NAN) == NAN) {
    // PREVENT MOTION HERE OF MOTOR
    /// @todo IF NO HOMES LISTED, SET A FLAG SO WE CAN"T DRIVE
    return;
  }

  // GET CURRENT VALUES
  units::degree_t frontLeft_current = units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition());
  units::degree_t frontRight_current = units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition());
  units::degree_t backRight_current = units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition());
  units::degree_t backLeft_current = units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition());

  // SUBTRACT SAVED FROM CURRENT
  const double frontLeftCalibrated = frontLeft_saved.to<double>() - frontLeft_current.to<double>();
  const double frontRightCalibrated = frontRight_saved.to<double>() - frontRight_current.to<double>();
  const double backRightCalibrated = backRight_saved.to<double>() - backRight_current.to<double>();
  const double backLeftCalibrated = backLeft_saved.to<double>() - backLeft_current.to<double>();

  // ASSIGN DIFFERENCE TO CURRENT MOTOR RELATIVE POSITION
  m_frontLeft.m_encoder.SetPosition(frontLeftCalibrated, 50);
  m_frontRight.m_encoder.SetPosition(frontRightCalibrated, 50);
  m_backRight.m_encoder.SetPosition(backRightCalibrated, 50);
  m_backLeft.m_encoder.SetPosition(backLeftCalibrated, 50);
}

// SWERVE MODULE SUBSYSTEM FUNCTIONS
SwerveModule::SwerveModule(const char driveAddr, const char turnAddr, const char encoderAddr)

    : m_drive(driveAddr), m_turn(turnAddr), m_encoder(encoderAddr) {}
