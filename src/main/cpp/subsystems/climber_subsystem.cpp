/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"

ClimberSubsystem::ClimberSubsystem()
    : m_motorLiftRight(address::climber::liftRight)
    , m_motorLiftLeft(address::climber::liftLeft)
    , m_motorMoveHook(address::climber::moveHook) {
  argos_lib::falcon_config::FalconConfig<motorConfig::climber::liftRight>(m_motorLiftRight, 50_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::climber::liftLeft>(m_motorLiftLeft, 50_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::climber::moveHook>(m_motorMoveHook, 50_ms);
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ArmReady() {}

void ClimberSubsystem::HookExtend() {}

void ClimberSubsystem::LowerBody() {}

void ClimberSubsystem::ArmToBar() {}

void ClimberSubsystem::BodyUp() {}

void ClimberSubsystem::StartingPosition() {}

void ClimberSubsystem::ManualControl(double hookSpeed, double armSpeed) {
  m_motorLiftRight.Set(armSpeed);
  m_motorLiftLeft.Set(armSpeed);
  m_motorMoveHook.Set(hookSpeed);
}