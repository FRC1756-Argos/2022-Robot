/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <memory>

#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

class network_tables {
 private:
  /**
   * @brief Create & configure all swerveHomes table instances
   *
   */
  void InitSwerveTable();

 public:
  // Meant to contain all the initialization and configuration of table's instances
  network_tables();

  // MASTER INSTANCE
  nt::NetworkTableInstance m_NtInstance;  //< The root instance of the network tables
  // TABLES
  std::shared_ptr<nt::NetworkTable> m_swerveHomes =
      m_NtInstance.GetTable(networkTables::swerveHomes::tableKey);  //< Instance of swerveHomes table
  // swerveHomes ENTRIES
  nt::NetworkTableEntry m_flHome;  //< Front left module home position
  nt::NetworkTableEntry m_frHome;  //< Front right module home position
  nt::NetworkTableEntry m_brHome;  //< Back right module home position
  nt::NetworkTableEntry m_blHome;  //< Back left module home position
};
