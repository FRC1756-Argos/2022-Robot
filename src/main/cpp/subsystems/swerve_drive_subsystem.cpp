/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/swerve_drive_subsystem.h"

#include <units/angle.h>
#include <units/velocity.h>

#include <memory>

#include "Constants.h"

// test commit

SwerveDriveSubsystem::SwerveDriveSubsystem()
    : m_frontLeft(address::motor::frontLeftDrive, address::motor::frontLeftTurn, address::encoders::frontLeftEncoder)
    , m_frontRight(
          address::motor::frontRightDrive, address::motor::frontRightTurn, address::encoders::frontRightEncoder)
    , m_backRight(address::motor::backRightDrive, address::motor::backRightTurn, address::encoders::backRightEncoder)
    , m_backLeft(address::motor::backLeftDrive, address::motor::backLeftTurn, address::encoders::backLeftEncoder) {
  // create our translation objects

  /// @todo add these to the swerve module class?

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

// SEWRVE DRIVE SUBSYSTEM MEMBER FUNCTIONS
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
  }

  auto moduleStates = m_pSwerveDriveKinematics->ToSwerveModuleStates(speeds);

  /// @todo change all of these to go into a for-each in future

  /// @todo switch to argosLib optimize functions in time (create overload for meters per second?)
  moduleStates.at(0).Optimize(moduleStates.at(0),
                              units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition()));
  moduleStates.at(1).Optimize(moduleStates.at(1),
                              units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition()));
  moduleStates.at(2).Optimize(moduleStates.at(2),
                              units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition()));
  moduleStates.at(3).Optimize(moduleStates.at(3),
                              units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition()));

  // Give module state values to motors-----------------------------------------------------------------------------------
  /// @todo also put this in a for-each loop

  // FROMT LEFT
  m_frontLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                          moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>());

  /// @todo check with david if the degree to double conversion is valid

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

  // END GIVE MODULE STATES TO MOTORS ------------------------------------------------------------------------------------
}

// SWERVE MODULE SUBSYSTEM FUNCTIONS
SwerveModule::SwerveModule(const char driveAddr, const char turnAddr, const char encoderAddr)

    /// @todo add addresses to "constants.h"

    : m_drive(driveAddr), m_turn(turnAddr), m_encoder(encoderAddr) {}
