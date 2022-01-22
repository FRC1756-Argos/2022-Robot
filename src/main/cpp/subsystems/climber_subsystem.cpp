/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "Constants.h"
#include "argos_lib/config/falcon_config.h"

climber_subsystem::climber_subsystem()
    : m_motorliftright(address::climber::liftRight)
    , m_motorliftleft(address::climber::liftLeft)
    , m_motormovehook(address::climber::moveHook) {
  argos_lib::falcon_config::FalconConfig<motorConfig::climber::liftRight>(m_motorliftright, 50_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::climber::liftLeft>(m_motorliftleft, 50_ms);
  argos_lib::falcon_config::FalconConfig<motorConfig::climber::moveHook>(m_motormovehook, 50_ms);
}
// This method will be called once per scheduler run
void climber_subsystem::Periodic() {}

void climber_subsystem::armReady() {}

void climber_subsystem::hookExtend() {}

void climber_subsystem::lowerBody() {}

void climber_subsystem::armToBar() {}

void climber_subsystem::bodyUp() {}

void climber_subsystem::startingPosition() {}
