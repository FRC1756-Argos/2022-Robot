/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/nt_button_monitor.h"

#include <argos_lib/controller/xbox_controller.h>

NtButtonMonitor::NtButtonMonitor() : m_ntInstance{nt::NetworkTableInstance::GetDefault()} {}

void NtButtonMonitor::UpdateButton(argos_lib::XboxController::Button btn) {
  std::shared_ptr<nt::NetworkTable> table = m_ntInstance.GetTable("argos/debugging");
}
