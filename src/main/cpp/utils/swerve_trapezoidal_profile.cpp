/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/swerve_trapezoidal_profile.h"

#include <frc/smartdashboard/SmartDashboard.h>

SwerveTrapezoidalProfileSegment::SwerveTrapezoidalProfileSegment()
    : m_initialPosition{}
    , m_relativeTranslation{}
    , m_relativeRotation{}
    , m_linearProfile{frc::TrapezoidProfile<units::inches>::Constraints(),
                      frc::TrapezoidProfile<units::inches>::State{}}
    , m_rotationalProfile{frc::TrapezoidProfile<units::degrees>::Constraints(),
                          frc::TrapezoidProfile<units::degrees>::State{}}
    , m_motionAngle{units::math::atan2(m_relativeTranslation.Y(), m_relativeTranslation.X())} {}

SwerveTrapezoidalProfileSegment::SwerveTrapezoidalProfileSegment(
    const frc::Pose2d initialPosition,
    const frc::Translation2d relativeTranslation,
    const frc::Rotation2d relativeRotation,
    const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints,
    const frc::TrapezoidProfile<units::degrees>::Constraints rotationalConstraints)
    : m_initialPosition{initialPosition}
    , m_relativeTranslation{relativeTranslation}
    , m_relativeRotation{relativeRotation}
    , m_linearProfile{linearConstraints, frc::TrapezoidProfile<units::inches>::State{relativeTranslation.Norm(), 0_mps}}
    , m_rotationalProfile{rotationalConstraints,
                          frc::TrapezoidProfile<units::degrees>::State{relativeRotation.Degrees(), 0_rpm}}
    , m_motionAngle{units::math::atan2(m_relativeTranslation.Y(), m_relativeTranslation.X())} {
  frc::SmartDashboard::PutNumber("(SwerveFollower) Relative X", units::inch_t{m_relativeTranslation.X()}.to<double>());
  frc::SmartDashboard::PutNumber("(SwerveFollower) Relative Y", units::inch_t{m_relativeTranslation.Y()}.to<double>());
  frc::SmartDashboard::PutNumber("(SwerveFollower) Relative Angle", m_relativeRotation.Degrees().to<double>());
}

frc::Trajectory::State SwerveTrapezoidalProfileSegment::Calculate(units::second_t time) const {
  const auto linearState = m_linearProfile.Calculate(time);
  const auto rotationalState = m_rotationalProfile.Calculate(time);
  const auto newTranslation =
      m_relativeTranslation * (linearState.position / m_relativeTranslation.Norm()).to<double>();
  frc::SmartDashboard::PutNumber("(SwerveFollower) Linear Position", linearState.position.to<double>());
  frc::SmartDashboard::PutNumber("(SwerveFollower) Linear Length",
                                 units::inch_t{m_relativeTranslation.Norm()}.to<double>());
  frc::SmartDashboard::PutNumber("(SwerveFollower) Completion %",
                                 (linearState.position / m_relativeTranslation.Norm()).to<double>());
  // const auto newRotation = m_relativeRotation * (rotationalState.position / m_relativeRotation.Degrees()).to<double>();
  return frc::Trajectory::State{time,
                                linearState.velocity,
                                0_mps_sq,
                                frc::Pose2d{m_initialPosition.Translation() + newTranslation, m_motionAngle},
                                units::curvature_t{rotationalState.velocity / linearState.velocity}};
}

bool SwerveTrapezoidalProfileSegment::IsFinished(units::second_t time) const {
  return m_linearProfile.IsFinished(time) && m_rotationalProfile.IsFinished(time);
}

frc::Rotation2d SwerveTrapezoidalProfileSegment::GetEndAngle() const {
  return m_initialPosition.Rotation() + m_relativeRotation;
}

units::feet_per_second_t SwerveTrapezoidalProfileSegment::GetXVelocity(const frc::Trajectory::State& state) const {
  return state.velocity * units::math::cos(m_motionAngle);
}

units::feet_per_second_t SwerveTrapezoidalProfileSegment::GetYVelocity(const frc::Trajectory::State& state) const {
  return state.velocity * units::math::sin(m_motionAngle);
}

SwerveTrapezoidalProfile::SwerveTrapezoidalProfile() = default;
