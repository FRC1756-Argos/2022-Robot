/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <memory>
#include <string>

#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"

class NetworkTablesWrapper {
 private:
  /**
   * @brief Create & configure all swerveHomes table instances
   *
   */
  void InitSwerveTable();

 public:
  /**
   * @brief Set the Entry Degrees object with an angle
   *
   * @param entryKey The key value for the NetworkTableEntry
   * @param degrees The angle value in degrees to write to the NetworkTableEntry
   */
  void SetEntryDegrees(const std::string& key, const units::degree_t degrees);

  /**
   * @brief Get the entry's angle value
   *
   * @param entry Reference to the network table entry
   * @return std::optional<units::degree_t> Standard library optional value
   */
  std::optional<units::degree_t> GetEntryDegrees(const std::string& key);

  // Meant to contain all the initialization and configuration of table's instances
  NetworkTablesWrapper();

  // MASTER INSTANCE
  nt::NetworkTableInstance m_NtInstance;  //< The root instance of the network tables
  // TABLES
  std::shared_ptr<nt::NetworkTable> m_swerveHomes;  //< Instance of swerveHomes table
  // swerveHomes ENTRIES
  nt::NetworkTableEntry m_flHome;  //< Front left module home position
  nt::NetworkTableEntry m_frHome;  //< Front right module home position
  nt::NetworkTableEntry m_brHome;  //< Back right module home position
  nt::NetworkTableEntry m_blHome;  //< Back left module home position
};
