/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/network_tables_wrapper.h"

// HELPER FUNCTIONS
void NetworkTablesWrapper::SetEntryDegrees(nt::NetworkTableEntry& entry, const units::degree_t degrees) {
  entry.SetDouble(degrees.to<double>());
}

std::optional<units::degree_t> NetworkTablesWrapper::GetEntryDegrees(nt::NetworkTableEntry& entry) {
  double savedAngle = entry.GetDouble(NAN);
  if (savedAngle == NAN) {
    return std::nullopt;
  } else {
    return std::optional<units::degree_t>{savedAngle};
  }
}

void NetworkTablesWrapper::InitSwerveTable() {
  m_flHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::flHome);
  m_flHome.SetPersistent();

  m_frHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::frHome);
  m_frHome.SetPersistent();

  m_brHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::brHome);
  m_brHome.SetPersistent();

  m_blHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::blHome);
  m_blHome.SetPersistent();
}

NetworkTablesWrapper::NetworkTablesWrapper() {
  // PUT ALL TABLE INIT FUNCTIONS HERE
  InitSwerveTable();
}
