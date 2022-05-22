/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "frc/geometry/Translation2d.h"
#include "units/angle.h"

namespace Framer {
  // TODO Make sure RefFrame can be constexpr
  /**
 * @brief Wraps a reference frame with useful functions for converting between one another
 *
 */
  class RefFrame {
   public:
    /**
   * @brief Construct a new Ref Frame object
   *
   * @param rootOffset The offset from the previously-decided frame of reference (recommend Stdandard Position)
   */
    explicit RefFrame(const units::angle::degree_t rootOffset);
    /**
   * @brief Get the Root Offset object
   *
   * @return units::angle::degree_t
   */
    units::angle::degree_t GetRootOffset() const;
    /**
   * @brief Converts an angle from this reference frame to another
   *
   * @param destination The reference frame the input angle is converting into
   * @param angle The angle to convert to the destination reference frame
   * @return units::angle::degree_t
   */
    units::angle::degree_t ConvertAngle(const RefFrame& destination, const units::degree_t& angle);
    /**
   * @brief Converts a point from this reference frame to another
   *
   * @param destination The reference frame the input point is converting into
   * @param point The point to convert to the destination reference frame
   * @return frc::Translation2d
   */
    frc::Translation2d ConvertPoint(const RefFrame& destination, const frc::Translation2d& point);
    frc::Translation2d ConvertPoint(const RefFrame& destination, const double x, const double y);
    /**
   * @brief Determines if the current reference frame's positive axis is CCW or CW
   *
   * @return true -> CCW+
   * @return false -> CW+
   */
    bool IsCCW() const;

   private:
    units::angle::degree_t m_rootOffset;  ///< The offset from the root reference frame
  };

  /**
 * @brief Static class containing functions for converting between reference frames
 *
 */
  class FoRConverter {
   public:
    /**
  * @brief Gets the "Beta" angle. The CCW+ angular difference between the destination reference frame and the root reference frame
  *
  * @param root The root reference frame
  * @param destination The destination reference frame
  * @return units::angle::degree_t
  */
    static units::angle::degree_t GetFrameOffset(const RefFrame& root, const RefFrame& destination);
    /**
   * @brief Get the angle equivalent (direction wise) to the input angle in the destination reference frame
   *
   * @param root The root reference frame
   * @param destination The destination reference frame
   * @param angle The angle to convert to the destination reference frame
   * @return units::angle::degree_t
   */
    static units::angle::degree_t GetRelativeAngle(const RefFrame& root,
                                                   const RefFrame& destination,
                                                   units::degree_t angle);
    /**
   * @brief Get the point equivalent (position wise) to the input angle in the destination reference frame
   *
   * @param root The root reference frame
   * @param destination The destination reference frame
   * @param point The point to convert to the destination reference frame
   * @return frc::Translation2d
   */
    static frc::Translation2d GetRelativePoint(const RefFrame& root,
                                               const RefFrame& destination,
                                               frc::Translation2d point);
    static frc::Translation2d GetRelativePoint(const RefFrame& root,
                                               const RefFrame& destination,
                                               const double x,
                                               const double y);

    /**
   * @brief Returns a home to save and use later, as a offset from a hardware position to a virtual position
   *
   * @param hardwareValue An absolute hardware value to offset from
   * @param virtualValue The angle currently homing too
   * @return units::angle::degree_t -> Value to save
   */
    static units::angle::degree_t CalculateHome(units::degree_t hardwareValue, units::degree_t virtualValue);
    /**
   * @brief Retreive the current angle of an absolute hardware value
   *
   * @param hardwareValue The current absolute hardware position
   * @param homedValue The previously saved home value
   * @return units::angle::degree_t -> Value to initialize to
   */
    static units::angle::degree_t InitHome(units::degree_t hardwareValue, units::degree_t homedValue);
  };

  /**
 * @brief Utility functions to support FoRConverter and RefFrame
 *
 */
  class Utils {
   public:
    /**
  * @brief Get the opposite angle of any given angle. ex: 90's opposite is 270
  *
  * @param angle Input angle to get opposite from
  * @return units::angle::degree_t
  */
    static units::angle::degree_t GetOppositeAngle(const units::degree_t angle);
    /**
   * @brief Constrain an angle between two values
   *
   * @param inVal The value to constrain
   * @param minVal The lower limit
   * @param maxVal The upper limit
   * @return units::angle::degree_t
   */
    static units::angle::degree_t ConstrainAngle(units::degree_t inVal, units::degree_t minVal, units::degree_t maxVal);
    /**
   * @brief Constrains an angle between 0 and 360 degrees
   *
   * @param inVal The value to constrain
   * @return units::angle::degree_t
   */
    static units::angle::degree_t ConstrainAngle(units::degree_t inVal);
  };

  /**
 * @brief Standard reference frames that might be commonly used
 *
 */
  namespace StdFrames {
    const RefFrame StdPos{360_deg};
    const RefFrame WPIStd{90_deg};
  }  // namespace StdFrames
}  // namespace Framer
