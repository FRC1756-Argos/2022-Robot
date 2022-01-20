/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include "Constants.h"
#include "argos_lib/general/interpolation.h"
#include "argos_lib/subsystems/swappable_controllers_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  argos_lib::InterpolationMap<decltype(controllerMap::driveLongSpeed.front().inVal),
                              controllerMap::driveLongSpeed.size()>
      m_driveLonSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveLatSpeed.front().inVal),
                              controllerMap::driveLongSpeed.size()>
      m_driveLatSpeedMap;

  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()>
      m_driveRotSpeed;
  argos_lib::SwappableControllersSubsystem m_controllers;

 private:
  // The robot's subsystems and commands are defined here...
  SwerveDriveSubsystem m_swerveDrive;

  void ConfigureButtonBindings();
};
