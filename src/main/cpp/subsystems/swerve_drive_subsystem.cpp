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
    : m_controlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl)
    , m_frontLeft(address::drive::frontLeftDrive, address::drive::frontLeftTurn, address::encoders::frontLeftEncoder)
    , m_frontRight(
          address::drive::frontRightDrive, address::drive::frontRightTurn, address::encoders::frontRightEncoder)
    , m_backRight(address::drive::backRightDrive, address::drive::backRightTurn, address::encoders::backRightEncoder)
    , m_backLeft(address::drive::backLeftDrive, address::drive::backLeftTurn, address::encoders::backLeftEncoder)
    , m_imu(frc::ADIS16448_IMU::kZ, frc::SPI::Port::kMXP, frc::ADIS16448_IMU::CalibrationTime::_4s)
    , m_swerveDriveKinematics(
          // Forward is positive X, left is positive Y
          // Front Left
          frc::Translation2d(measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontLeftLOffset,
                             measure_up::chassis::width / 2 - measure_up::swerve_offsets::frontLeftWOffset),
          // Front Right
          frc::Translation2d(measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontRightLOffset,
                             -measure_up::chassis::width / 2 + measure_up::swerve_offsets::frontRightWOffset),
          // Back Right
          frc::Translation2d(-measure_up::chassis::length / 2 + measure_up::swerve_offsets::backRightLOffset,
                             -measure_up::chassis::width / 2 + measure_up::swerve_offsets::backRightWOffset),
          // Back Left
          frc::Translation2d(-measure_up::chassis::length / 2 + measure_up::swerve_offsets::backLeftLOffset,
                             measure_up::chassis::width / 2 - measure_up::swerve_offsets::backLeftWOffset))
    , m_odometry(m_swerveDriveKinematics, frc::Rotation2d(m_imu.GetAngle()))
    , m_prevOdometryAngle{0_deg}
    , m_continuousOdometryOffset{0_deg}
    , m_pNetworkTable(networkTable)
    , m_fsStorage(paths::swerveHomesPath)
    , m_followingProfile(false)
    , m_pActiveSwerveProfile(nullptr)
    , m_swerveProfileStartTime()
    , m_linearPID(instance == argos_lib::RobotInstance::Competition ?
                      frc2::PIDController{controlLoop::comp_bot::drive::linear_follower::kP,
                                          controlLoop::comp_bot::drive::linear_follower::kI,
                                          controlLoop::comp_bot::drive::linear_follower::kD} :
                      frc2::PIDController{controlLoop::practice_bot::drive::linear_follower::kP,
                                          controlLoop::practice_bot::drive::linear_follower::kI,
                                          controlLoop::practice_bot::drive::linear_follower::kD})
    , m_rotationalPID(instance == argos_lib::RobotInstance::Competition ?
                          frc::ProfiledPIDController<units::radians>{
                              controlLoop::comp_bot::drive::rotational_follower::kP,
                              controlLoop::comp_bot::drive::rotational_follower::kI,
                              controlLoop::comp_bot::drive::rotational_follower::kD,
                              frc::TrapezoidProfile<units::radians>::Constraints(
                                  controlLoop::comp_bot::drive::rotational_follower::angularVelocity,
                                  controlLoop::comp_bot::drive::rotational_follower::angularAcceleration)} :
                          frc::ProfiledPIDController<units::radians>{
                              controlLoop::practice_bot::drive::rotational_follower::kP,
                              controlLoop::practice_bot::drive::rotational_follower::kI,
                              controlLoop::practice_bot::drive::rotational_follower::kD,
                              frc::TrapezoidProfile<units::radians>::Constraints(
                                  controlLoop::practice_bot::drive::rotational_follower::angularVelocity,
                                  controlLoop::practice_bot::drive::rotational_follower::angularAcceleration)})
    , m_followerController{m_linearPID, m_linearPID, m_rotationalPID}
    , m_driveMotorPIDTuner(
          "argos/drive/motors",
          {&m_frontLeft.m_drive, &m_frontRight.m_drive, &m_backRight.m_drive, &m_backLeft.m_drive},
          0,
          argos_lib::ClosedLoopSensorConversions{
              argos_lib::GetSensorConversionFactor(sensor_conversions::swerve_drive::drive::ToDistance),
              argos_lib::GetSensorConversionFactor(sensor_conversions::swerve_drive::drive::ToVelocity),
              argos_lib::GetSensorConversionFactor(sensor_conversions::swerve_drive::drive::ToVelocity)}) {
  // TURN MOTORS CONFIG
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

  InitializeMotors();
}

void SwerveDriveSubsystem::Disable() {
  m_controlMode = DriveControlMode::fieldCentricControl;
  m_followingProfile = false;
  StopDrive();
}

// SWERVE DRIVE SUBSYSTEM MEMBER FUNCTIONS

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetRawModuleStates(
    SwerveDriveSubsystem::Velocities velocities) {
  // IF SPEEDS ZERO, SET MOTORS TO ZERO AND RETURN
  if (velocities.fwVelocity == 0 && velocities.sideVelocity == 0 && velocities.rotVelocity == 0) {
    StopDrive();
    /// @todo fix later
    frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

    return m_swerveDriveKinematics.ToSwerveModuleStates(emptySpeeds);
  }

  switch (m_controlMode) {
    case (DriveControlMode::
              fieldCentricControl): {  // Construct speeds with field-relative speeds and current IMU Z angle.
      frc::ChassisSpeeds fieldCentricSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          units::make_unit<units::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity),
          frc::Rotation2d(GetFieldCentricAngle()));

      // Return the speeds to consumer
      return m_swerveDriveKinematics.ToSwerveModuleStates(fieldCentricSpeeds);
    }

    case (DriveControlMode::robotCentricControl): {
      // Construct speeds just the same as in the current main drive function
      frc::ChassisSpeeds robotCentricSpeeds{
          units::make_unit<units::velocity::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::velocity::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity)};

      return m_swerveDriveKinematics.ToSwerveModuleStates(robotCentricSpeeds);
    }
  }
  frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

  return m_swerveDriveKinematics.ToSwerveModuleStates(emptySpeeds);
}

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetCurrentModuleStates() {
  return {m_frontLeft.GetState(), m_frontRight.GetState(), m_backRight.GetState(), m_backLeft.GetState()};
}

void SwerveDriveSubsystem::SwerveDrive(const double& fwVelocity,
                                       const double& sideVelocity,
                                       const double& rotVelocity) {
  UpdateOdometry();
  if (fwVelocity == 0 && sideVelocity == 0 && rotVelocity == 0) {
    if (!m_followingProfile) {
      StopDrive();
      return;
    }
  } else {
    // Manual override
    m_followingProfile = false;
  }

  SwerveDriveSubsystem::Velocities velocities{fwVelocity, sideVelocity, rotVelocity};

  // DEBUG STUFF
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) fwVelocity", fwVelocity);
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) sideVelocity", sideVelocity);
  frc::SmartDashboard::PutNumber("(DRIVETRAIN) rotVelocity", rotVelocity);
  frc::SmartDashboard::PutNumber("CONTROL MODE", m_controlMode);
  frc::SmartDashboard::PutNumber("IMU ANGLE", m_imu.GetAngle().to<double>());

  // SET MODULES BASED OFF OF CONTROL MODE
  auto moduleStates = GetCurrentModuleStates();
  frc::Trajectory::State desiredProfileState;
  if (m_followingProfile && m_pActiveSwerveProfile) {
    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                   m_swerveProfileStartTime);
    if (!m_pActiveSwerveProfile->IsFinished(elapsedTime)) {
      desiredProfileState = m_pActiveSwerveProfile->Calculate(elapsedTime);
      const auto controllerChassisSpeeds = m_followerController.Calculate(
          GetContinuousOdometry(), desiredProfileState, m_pActiveSwerveProfile->GetEndAngle());
      // const auto controllerChassisSpeeds =  m_followerController.Calculate(GetContinuousOdometry(), desiredProfileState.pose, desiredProfileState.velocity, m_pActiveSwerveProfile->GetEndAngle());
      // const auto swappedChassisSpeeds = frc::ChassisSpeeds{controllerChassisSpeeds.vy, controllerChassisSpeeds.vx, controllerChassisSpeeds.omega};
      moduleStates = m_swerveDriveKinematics.ToSwerveModuleStates(controllerChassisSpeeds);
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired X",
                                     units::inch_t{desiredProfileState.pose.X()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Y",
                                     units::inch_t{desiredProfileState.pose.Y()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Angle",
                                     desiredProfileState.pose.Rotation().Degrees().to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Curvature",
                                     units::unit_t<units::compound_unit<units::degrees, units::inverse<units::feet>>>{
                                         desiredProfileState.curvature}
                                         .to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) End Angle",
                                     m_pActiveSwerveProfile->GetEndAngle().Degrees().to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Desired Vel",
                                     units::feet_per_second_t{desiredProfileState.velocity}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Current X",
                                     units::inch_t{GetContinuousOdometry().X()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Current Y",
                                     units::inch_t{GetContinuousOdometry().Y()}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Current Angle",
                                     GetContinuousOdometry().Rotation().Degrees().to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Controller Vx",
                                     units::feet_per_second_t{controllerChassisSpeeds.vx}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Controller Vy",
                                     units::feet_per_second_t{controllerChassisSpeeds.vy}.to<double>());
      frc::SmartDashboard::PutNumber("(SwerveFollower) Controller Omega",
                                     units::degrees_per_second_t{controllerChassisSpeeds.omega}.to<double>());
      // frc::SmartDashboard::PutNumber("(SwerveFollower) Current Vel", units::feet_per_second_t{desiredProfileState.velocity}.to<double>());
    } else {
      m_followingProfile = false;
    }
  } else if (m_followingProfile) {
    // Bad profile
    m_followingProfile = false;
  } else {
    moduleStates = GetRawModuleStates(velocities);
  }

  /// @todo Convert sensor velocities for optimizer instead of constants
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

  if (m_followingProfile) {
    // When following profile, use closed-loop velocity
    // FRONT LEFT
    m_frontLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                            sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                                moduleStates.at(indexes::swerveModules::frontLeftIndex).speed));

    m_frontLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           sensor_conversions::swerve_drive::turn::ToSensorUnit(
                               moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees()));

    // FRONT RIGHT
    m_frontRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                             sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                                 moduleStates.at(indexes::swerveModules::frontRightIndex).speed));

    m_frontRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                            sensor_conversions::swerve_drive::turn::ToSensorUnit(
                                moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees()));

    // BACK RIGHT
    m_backRight.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                            sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                                moduleStates.at(indexes::swerveModules::backRightIndex).speed));

    m_backRight.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                           sensor_conversions::swerve_drive::turn::ToSensorUnit(
                               moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees()));

    // BACK LEFT
    m_backLeft.m_drive.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                           sensor_conversions::swerve_drive::drive::ToSensorVelocity(
                               moduleStates.at(indexes::swerveModules::backLeftIndex).speed));

    m_backLeft.m_turn.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
                          sensor_conversions::swerve_drive::turn::ToSensorUnit(
                              moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees()));
  } else {
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
  }

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

void SwerveDriveSubsystem::FieldHome(units::degree_t homeAngle, bool updateOdometry) {
  m_fieldHomeOffset = -m_imu.GetAngle() - homeAngle;
  if (updateOdometry) {
    // Update odometry as well
    const auto currentPose = m_odometry.GetPose();
    m_odometry.ResetPosition(frc::Pose2d{currentPose.Translation(), frc::Rotation2d(homeAngle)}, -m_imu.GetAngle());
    m_prevOdometryAngle = m_odometry.GetPose().Rotation().Degrees();
    m_continuousOdometryOffset = 0_deg;
  }
}

void SwerveDriveSubsystem::InitializeOdometry(const frc::Pose2d& currentPose) {
  m_odometry.ResetPosition(currentPose, -m_imu.GetAngle());
  m_prevOdometryAngle = m_odometry.GetPose().Rotation().Degrees();
  m_continuousOdometryOffset = 0_deg;
  // Since we know the position, might as well update the driving orientation as well
  FieldHome(currentPose.Rotation().Degrees(), false);
}

frc::Rotation2d SwerveDriveSubsystem::GetContinuousOdometryAngle() {
  const auto latestOdometry = m_odometry.GetPose();

  if (m_prevOdometryAngle > 90_deg && latestOdometry.Rotation().Degrees() < -(90_deg)) {
    m_continuousOdometryOffset += 360_deg;
  } else if (m_prevOdometryAngle < -(90_deg) && latestOdometry.Rotation().Degrees() > 90_deg) {
    m_continuousOdometryOffset -= 360_deg;
  }
  m_prevOdometryAngle = latestOdometry.Rotation().Degrees();

  return frc::Rotation2d{latestOdometry.Rotation().Degrees() + m_continuousOdometryOffset};
}

frc::Pose2d SwerveDriveSubsystem::GetContinuousOdometry() {
  const auto discontinuousOdometry = m_odometry.GetPose();
  return frc::Pose2d{discontinuousOdometry.Translation(), GetContinuousOdometryAngle()};
}

frc::Pose2d SwerveDriveSubsystem::UpdateOdometry() {
  const auto newPose = m_odometry.Update(frc::Rotation2d{-m_imu.GetAngle()},
                                         m_frontLeft.GetState(),
                                         m_frontRight.GetState(),
                                         m_backRight.GetState(),
                                         m_backLeft.GetState());
  const auto continuousOdometryAngle = GetContinuousOdometryAngle();
  frc::SmartDashboard::PutNumber("(Odometry) X", units::inch_t{newPose.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Y", units::inch_t{newPose.Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Angle", newPose.Rotation().Degrees().to<double>());
  frc::SmartDashboard::PutNumber("(Odometry) Continuous Angle", continuousOdometryAngle.Degrees().to<double>());
  return frc::Pose2d{newPose.Translation(), continuousOdometryAngle};
}

units::degree_t SwerveDriveSubsystem::GetFieldCentricAngle() const {
  return -m_imu.GetAngle() - m_fieldHomeOffset;
}

frc::Pose2d SwerveDriveSubsystem::GetPoseEstimate() {
  return GetContinuousOdometry();
}

void SwerveDriveSubsystem::SetControlMode(SwerveDriveSubsystem::DriveControlMode controlMode) {
  m_controlMode = controlMode;
}

void SwerveDriveSubsystem::InitializeMotors() {
  InitializeMotorsFromFS();
}

void SwerveDriveSubsystem::HomeToNetworkTables(const units::degree_t& angle) {
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

  m_pNetworkTable->SetEntryDegrees(networkTables::swerveHomes::keys::flHome, homes.FrontLeft);
  m_pNetworkTable->SetEntryDegrees(networkTables::swerveHomes::keys::frHome, homes.FrontRight);
  m_pNetworkTable->SetEntryDegrees(networkTables::swerveHomes::keys::brHome, homes.RearRight);
  m_pNetworkTable->SetEntryDegrees(networkTables::swerveHomes::keys::blHome, homes.RearLeft);
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

void SwerveDriveSubsystem::InitializeMotorsFromNetworkTables() {
  // GET SAVED VALUES
  std::optional<units::degree_t> frontLeft_saved =
      m_pNetworkTable->GetEntryDegrees(networkTables::swerveHomes::keys::flHome);
  std::optional<units::degree_t> frontRight_saved =
      m_pNetworkTable->GetEntryDegrees(networkTables::swerveHomes::keys::frHome);
  std::optional<units::degree_t> backRight_saved =
      m_pNetworkTable->GetEntryDegrees(networkTables::swerveHomes::keys::brHome);
  std::optional<units::degree_t> backLeft_saved =
      m_pNetworkTable->GetEntryDegrees(networkTables::swerveHomes::keys::blHome);

  if (!frontLeft_saved || !frontRight_saved || !backRight_saved || !backLeft_saved) {
    // @todo PREVENT MOTION HERE OF MOTOR
    /// @todo IF NO HOMES LISTED, SET A FLAG SO WE CAN'T DRIVE
    return;
  }

  // GET CURRENT VALUES
  units::degree_t frontLeft_current = units::make_unit<units::degree_t>(m_frontLeft.m_encoder.GetAbsolutePosition());
  units::degree_t frontRight_current = units::make_unit<units::degree_t>(m_frontRight.m_encoder.GetAbsolutePosition());
  units::degree_t backRight_current = units::make_unit<units::degree_t>(m_backRight.m_encoder.GetAbsolutePosition());
  units::degree_t backLeft_current = units::make_unit<units::degree_t>(m_backLeft.m_encoder.GetAbsolutePosition());

  // SUBTRACT SAVED FROM CURRENT
  const units::degree_t frontLeftCalibrated = frontLeft_current - frontLeft_saved.value();
  const units::degree_t frontRightCalibrated = frontRight_current - frontRight_saved.value();
  const units::degree_t backRightCalibrated = backRight_current - backRight_saved.value();
  const units::degree_t backLeftCalibrated = backLeft_current - backLeft_saved.value();

  // ASSIGN DIFFERENCE TO CURRENT MOTOR RELATIVE POSITION
  m_frontLeft.m_encoder.SetPosition(frontLeftCalibrated.to<double>(), 50);
  m_frontRight.m_encoder.SetPosition(frontRightCalibrated.to<double>(), 50);
  m_backRight.m_encoder.SetPosition(backRightCalibrated.to<double>(), 50);
  m_backLeft.m_encoder.SetPosition(backLeftCalibrated.to<double>(), 50);
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

frc::SwerveModuleState SwerveModule::GetState() {
  return frc::SwerveModuleState{
      sensor_conversions::swerve_drive::drive::ToVelocity(m_drive.GetSelectedSensorVelocity()),
      frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(m_turn.GetSelectedSensorPosition())}};
}

void SwerveDriveSubsystem::UpdateFollowerLinearPIDParams(double kP, double kI, double kD) {
  m_linearPID.SetPID(kP, kI, kD);
  m_linearPID.Reset();
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDParams(double kP, double kI, double kD) {
  m_rotationalPID.SetPID(kP, kI, kD);
  m_rotationalPID.Reset(m_rotationalPID.GetGoal());
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDConstraints(
    frc::TrapezoidProfile<units::degrees>::Constraints constraints) {
  m_rotationalPID.SetConstraints(
      frc::TrapezoidProfile<units::radians>::Constraints{constraints.maxVelocity, constraints.maxAcceleration});
  m_rotationalPID.Reset(m_rotationalPID.GetGoal());
}

void SwerveDriveSubsystem::StartDrivingProfile(SwerveTrapezoidalProfileSegment newProfile) {
  m_pActiveSwerveProfile = std::make_unique<SwerveTrapezoidalProfileSegment>(newProfile);
  m_followerController = frc::HolonomicDriveController(m_linearPID, m_linearPID, m_rotationalPID);
  m_swerveProfileStartTime = std::chrono::steady_clock::now();
  m_followingProfile = true;
}

void SwerveDriveSubsystem::CancelDrivingProfile() {
  m_followingProfile = false;
}
