/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/swerve_trapezoidal_profile.h"

SwerveTrapezoidalProfileSegment::SwerveTrapezoidalProfileSegment()
    : m_initialPosition{}
    , m_relativeTranslation{}
    , m_relativeRotation{}
    , m_linearProfile{frc::TrapezoidProfile<units::inches>::Constraints(),
                      frc::TrapezoidProfile<units::inches>::State{}}
    , m_rotationalProfile{frc::TrapezoidProfile<units::degrees>::Constraints(),
                          frc::TrapezoidProfile<units::degrees>::State{}} {}

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
                          frc::TrapezoidProfile<units::degrees>::State{relativeRotation.Degrees(), 0_rpm}} {}

frc::Pose2d SwerveTrapezoidalProfileSegment::Calculate(units::second_t time) const {
  const auto newTranslation =
      m_relativeTranslation * (m_linearProfile.Calculate(time).position() / m_relativeTranslation.Norm()).to<double>();
  const auto newRotation =
      m_relativeRotation * (m_rotationalProfile.Calculate(time).position() / m_relativeRotation.Degrees()).to<double>();
  return frc::Pose2d(m_initialPosition.Translation() + newTranslation, m_initialPosition.Rotation() + newRotation);
}

bool SwerveTrapezoidalProfileSegment::IsFinished(units::second_t time) const {
  return m_linearProfile.IsFinished(time) && m_rotationalProfile.IsFinished(time);
}

SwerveTrapezoidalProfile::SwerveTrapezoidalProfile() = default;
