/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <units/length.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

//TODO rename namespaces to have snake case(lower)


namespace address{
  namespace motor{
    constexpr const char frontLeftDrive = NULL;
    constexpr const char frontLeftTurn = NULL;
    constexpr const char frontRightDrive = NULL;
    constexpr const char frontRightTurn = NULL;
    constexpr const char backRightDrive = NULL;
    constexpr const char backRightTurn = NULL;
    constexpr const char backLeftDrive = NULL;
    constexpr const char backLeftTurn = NULL;
    }
  namespace encoders
  {
    constexpr const char frontLeftEncoder = NULL;
    constexpr const char frontRightEncoder = NULL;
    constexpr const char backRightEncoder = NULL;
    constexpr const char backLeftEncoder = NULL;
  }


}

namespace measureUp {
  namespace chassis {
    constexpr units::inch_t width{28.0};
    constexpr units::inch_t length{31.0};
  }  // namesp
  namespace swerveOffsets {
    constexpr auto frontLeftLOffset = 4.0_in;
    constexpr auto frontLeftWOffset = 4.0_in;
    constexpr auto frontRightLOffset = 4.0_in;
    constexpr auto frontRightWOffset = 4.0_in;
    constexpr auto backRightWOffset = 4.0_in;
    constexpr auto backRightLOffset = 4.0_in;
    constexpr auto backLeftWOffset = 4.0_in;
    constexpr auto backLeftLOffset = 4.0_in;
  }
}
