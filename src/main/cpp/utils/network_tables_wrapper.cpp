/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/network_tables_wrapper.h"

// HELPER FUNCTIONS
void NetworkTablesWrapper::SetEntryDegrees(const std::string& key, const units::degree_t degrees) {
  m_swerveHomes->PutNumber(key, degrees.to<double>());
}

std::optional<units::degree_t> NetworkTablesWrapper::GetEntryDegrees(const std::string& key) {
  double savedAngle = m_swerveHomes->GetNumber(key, NAN);
  if (savedAngle == NAN) {
    return std::nullopt;
  } else {
    return std::optional<units::degree_t>{savedAngle};
  }
}

void NetworkTablesWrapper::InitSwerveTable() {
  std::printf("%d\n", __LINE__);
  m_swerveHomes->SetPersistent(networkTables::swerveHomes::keys::flHomeFullPath);
  m_swerveHomes->SetPersistent(networkTables::swerveHomes::keys::frHomeFullPath);
  m_swerveHomes->SetPersistent(networkTables::swerveHomes::keys::brHomeFullPath);
  m_swerveHomes->SetPersistent(networkTables::swerveHomes::keys::blHomeFullPath);

  // m_flHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::flHome);
  // m_frHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::frHome);
  // m_brHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::brHome);
  // m_blHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::blHome);

  std::printf("%d\n", __LINE__);
}

NetworkTablesWrapper::NetworkTablesWrapper() {
  std::printf("%d\n", __LINE__);
  m_NtInstance = nt::NetworkTableInstance::GetDefault();
  m_swerveHomes = m_NtInstance.GetTable(networkTables::swerveHomes::tableKey);
  // PUT ALL TABLE INIT FUNCTIONS HERE
  InitSwerveTable();
}
