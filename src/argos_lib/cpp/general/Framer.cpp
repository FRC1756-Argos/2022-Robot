/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/Framer.h"

#include "units/angle.h"
#include "units/math.h"

// ================================================ REFERENCE FRAME CLASS
// ================================================

Framer::RefFrame::RefFrame(const units::angle::degree_t rootOffset) {
  if (rootOffset == 0_deg) {
    m_rootOffset = 360_deg;
    return;
  }
  m_rootOffset = rootOffset;
}

units::degree_t Framer::RefFrame::GetRootOffset() const {
  return m_rootOffset;
}

units::degree_t Framer::RefFrame::ConvertAngle(const RefFrame& destination, const units::degree_t& angle) {
  return FoRConverter::GetRelativeAngle(*this, destination, angle);
}

frc::Translation2d Framer::RefFrame::ConvertPoint(const RefFrame& destination, const frc::Translation2d& point) {
  return FoRConverter::GetRelativePoint(*this, destination, point);
}

frc::Translation2d Framer::RefFrame::ConvertPoint(const RefFrame& destination, const double x, const double y) {
  return FoRConverter::GetRelativePoint(*this, destination, x, y);
}

bool Framer::RefFrame::IsCCW() const {
  return (this->GetRootOffset() > 0_rad) ? true : false;
}

// ================================================ FRAME CONVERTER
// ================================================

units::degree_t Framer::FoRConverter::GetFrameOffset(const RefFrame& root, const RefFrame& destination) {
  return Utils::ConstrainAngle(destination.GetRootOffset() - root.GetRootOffset(), 0_deg, 360_deg);
}

units::degree_t Framer::FoRConverter::GetRelativeAngle(const RefFrame& root,
                                                       const RefFrame& destination,
                                                       units::degree_t angle) {
  units::degree_t frameOffset = GetFrameOffset(root, destination);
  if (root.IsCCW() && destination.IsCCW()) {
    return Utils::ConstrainAngle(angle - frameOffset);
  } else if (root.IsCCW() && !destination.IsCCW()) {
    return Utils::ConstrainAngle(Utils::GetOppositeAngle(angle) - Utils::GetOppositeAngle(frameOffset));
  } else if (!root.IsCCW() && !destination.IsCCW()) {
    return Utils::ConstrainAngle(angle - Utils::GetOppositeAngle(frameOffset));
  } else {
    return Utils::ConstrainAngle(Utils::GetOppositeAngle(angle) - frameOffset);
  }
}

frc::Translation2d Framer::FoRConverter::GetRelativePoint(const RefFrame& root,
                                                          const RefFrame& destination,
                                                          frc::Translation2d point) {
  return point.RotateBy(frc::Rotation2d{Utils::GetOppositeAngle(GetFrameOffset(root, destination))});
}

frc::Translation2d Framer::FoRConverter::GetRelativePoint(const RefFrame& root,
                                                          const RefFrame& destination,
                                                          const double x,
                                                          const double y) {
  frc::Translation2d convertedPoint = FoRConverter::GetRelativePoint(
      root, destination, frc::Translation2d{units::make_unit<units::meter_t>(x), units::make_unit<units::meter_t>(y)});
  return convertedPoint;
}

units::angle::degree_t Framer::FoRConverter::CalculateHome(units::degree_t hardwareValue,
                                                           units::degree_t virtualValue) {
  return hardwareValue - Utils::ConstrainAngle(virtualValue);
}
units::angle::degree_t Framer::FoRConverter::InitHome(units::degree_t hardwareValue, units::degree_t homedValue) {
  return Utils::ConstrainAngle(hardwareValue - homedValue);
}

// ================================================ UTILS FUNCTIONS
// ================================================

units::degree_t Framer::Utils::ConstrainAngle(units::degree_t inVal, units::degree_t minVal, units::degree_t maxVal) {
  const auto range = maxVal - minVal;
  inVal = units::math::fmod(inVal - minVal, maxVal - minVal);
  if (inVal < 0_deg) {
    inVal += range;
  }
  return inVal + minVal;
}
units::degree_t Framer::Utils::ConstrainAngle(units::degree_t inVal) {
  return ConstrainAngle(inVal, 0_deg, 360_deg);
}

units::degree_t Framer::Utils::GetOppositeAngle(const units::degree_t angle) {
  return Utils::ConstrainAngle(360_deg - angle, 0_deg, 360_deg);
}
