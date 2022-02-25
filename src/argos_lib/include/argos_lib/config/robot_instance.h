/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

namespace argos_lib {
  /**
   * @brief Differentiates between practice robot and competition robot
   */
  enum struct RobotInstance {
    Competition,  ///< Competition robot
    Practice      ///< Practice robot
  };

  /**
   * @brief Detect robot instance
   *
   * @return RobotInstance
   */
  RobotInstance GetRobotInstance();
}  // namespace argos_lib
