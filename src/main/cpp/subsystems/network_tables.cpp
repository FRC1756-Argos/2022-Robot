/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/network_tables.h"

void network_tables::InitSwerveTable() {
  m_flHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::flHome);
  m_flHome.SetPersistent();

  m_frHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::frHome);
  m_frHome.SetPersistent();

  m_brHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::brHome);
  m_brHome.SetPersistent();

  m_blHome = m_swerveHomes->GetEntry(networkTables::swerveHomes::keys::blHome);
  m_blHome.SetPersistent();
}

network_tables::network_tables() {
  // PUT ALL TABLE INIT FUNCTIONS HERE
  InitSwerveTable();
}
