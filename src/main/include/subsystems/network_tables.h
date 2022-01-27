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
 public:
  network_tables();

  // MASTER INSTANCE
  nt::NetworkTableInstance table_instance;

  // TABLES
  std::shared_ptr<nt::NetworkTable> swerveHomes = table_instance.GetTable(networkTables::swerveHomes::tableKey);
  // swerveHomes ENTRIES
  nt::NetworkTableEntry flHome = swerveHomes->GetEntry(networkTables::swerveHomes::keys::flHome);
  nt::NetworkTableEntry frHome = swerveHomes->GetEntry(networkTables::swerveHomes::keys::frHome);
  nt::NetworkTableEntry brHome = swerveHomes->GetEntry(networkTables::swerveHomes::keys::brHome);
  nt::NetworkTableEntry blHome = swerveHomes->GetEntry(networkTables::swerveHomes::keys::blHome);
};
