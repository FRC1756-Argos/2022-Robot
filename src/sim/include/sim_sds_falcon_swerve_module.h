/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/length.h>

#include "ctre/Phoenix.h"

struct SDSModuleConfig {
  units::inch_t wheelDiameter;
  double gearRatioDrive;
  double gearRatioTurn;
};

class SimSDSFalconSwerveModule {
 public:
 private:
  WPI_TalonFX& m_driveMotor;
  WPI_TalonFX& m_turnMotor;
  CANCoder& m_turnEncoder;

  TalonFXSimCollection m_simDriveMotor;
  TalonFXSimCollection m_simTurnMotor;
  CANCoderSimCollection m_simTurnEncoder;
};
