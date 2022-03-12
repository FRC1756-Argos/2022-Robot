/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

enum class SegmentType { start, middle, end };

class SwerveTrapezoidalProfileSegment {
 public:
  SwerveTrapezoidalProfileSegment();
  SwerveTrapezoidalProfileSegment(const frc::Pose2d initialPosition,
                                  const frc::Translation2d relativeTranslation,
                                  const frc::Rotation2d relativeRotation,
                                  const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints);
  SwerveTrapezoidalProfileSegment(const frc::Pose2d initialPosition,
                                  const frc::Pose2d finalPosition,
                                  const frc::TrapezoidProfile<units::inches>::Constraints linearConstraints);
  SwerveTrapezoidalProfileSegment(const SwerveTrapezoidalProfileSegment& other) = default;
  SwerveTrapezoidalProfileSegment(SwerveTrapezoidalProfileSegment&& other) = default;

  SwerveTrapezoidalProfileSegment& operator=(const SwerveTrapezoidalProfileSegment&) = delete;
  SwerveTrapezoidalProfileSegment& operator=(SwerveTrapezoidalProfileSegment&&) = delete;

  frc::Trajectory::State Calculate(units::second_t time) const;
  bool IsFinished(units::second_t time) const;
  frc::Rotation2d GetEndAngle() const;

  units::feet_per_second_t GetXVelocity(const frc::Trajectory::State& state) const;
  units::feet_per_second_t GetYVelocity(const frc::Trajectory::State& state) const;

 private:
  const frc::Pose2d m_initialPosition;
  const frc::Translation2d m_relativeTranslation;
  const frc::Rotation2d m_relativeRotation;
  const frc::TrapezoidProfile<units::inches> m_linearProfile;
  const units::degree_t m_motionAngle;
};

class SwerveTrapezoidalProfile {
 public:
  SwerveTrapezoidalProfile();
};
