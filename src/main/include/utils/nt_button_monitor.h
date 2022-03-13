/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <argos_lib/controller/xbox_controller.h>
#include <networktables/NetworkTableInstance.h>

#include <string>

class NtButtonMonitor {
 public:
  NtButtonMonitor();

  void UpdateButton(argos_lib::XboxController::Button btn);

 private:
  nt::NetworkTableInstance m_ntInstance;
};
