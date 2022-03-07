/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/Compressor.h>
#include <frc2/command/Command.h>

#include <memory>

#include "Constants.h"
#include "argos_lib/config/robot_instance.h"
#include "argos_lib/general/interpolation.h"
#include "argos_lib/general/nt_subscriber.h"
#include "argos_lib/subsystems/swappable_controllers_subsystem.h"
#include "commands/home_climber_arm_command.h"
#include "commands/home_climber_hook_command.h"
#include "commands/home_hood_command.h"
#include "subsystems/climber_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "utils/network_tables_wrapper.h"
#include "utils/sensor_conversions.h"

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

  /**
   * @brief Called once when robot is disabled
   */
  void Disable();

 private:
  std::shared_ptr<NetworkTablesWrapper> m_pNetworkTable;

  // The robot's subsystems and commands are defined here...
  argos_lib::InterpolationMap<decltype(controllerMap::driveLongSpeed.front().inVal),
                              controllerMap::driveLongSpeed.size()>
      m_driveLonSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveLatSpeed.front().inVal),
                              controllerMap::driveLongSpeed.size()>
      m_driveLatSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()>
      m_driveRotSpeed;

  argos_lib::InterpolationMap<decltype(controllerMap::hookSpeed.front().inVal), controllerMap::hookSpeed.size()>
      m_hookSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::armSpeed.front().inVal), controllerMap::armSpeed.size()>
      m_armSpeedMap;

  argos_lib::InterpolationMap<decltype(controllerMap::turretSpeed.front().inVal), controllerMap::turretSpeed.size()>
      m_turretSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::hoodSpeed.front().inVal), controllerMap::hoodSpeed.size()>
      m_hoodSpeedMap;

  const argos_lib::RobotInstance m_instance;

  argos_lib::SwappableControllersSubsystem m_controllers;
  SwerveDriveSubsystem m_swerveDrive;
  IntakeSubsystem m_intake;
  std::unique_ptr<ClimberSubsystem> m_pClimber;
  ShooterSubsystem m_shooter;

  HomeHoodCommand m_homeHoodCommand;
  HomeClimberArmCommand m_homeClimberArmCommand;
  HomeClimberHookCommand m_homeClimberHookCommand;

  // Tuning stuff
  units::degree_t m_hoodTargetPosition;
  units::revolutions_per_minute_t m_shooterTargetVelocity;
  units::inch_t m_climberArmTargetExtension;
  units::inches_per_second_t m_climberArmCruiseVelocity;
  units::inches_per_second_squared_t m_climberArmAccel;
  units::inch_t m_climberHookTargetExtension;
  units::inches_per_second_t m_climberHookCruiseVelocity;
  units::inches_per_second_squared_t m_climberHookAccel;
  units::degree_t m_turretTargetPosition;
  argos_lib::NTSubscriber m_NTMonitor;

  void ConfigureButtonBindings();
};
