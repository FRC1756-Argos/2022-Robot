// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>
#include <iostream>
#include "subsystems/lights_subsystem.h"
#include "ctre/phoenix/led/CANdleConfiguration.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"

LightsSubsystem::LightsSubsystem(u_int numLeds, u_char addr, std::string canBus) : m_candle{addr}, m_numLeds{numLeds} {
  CANdleConfiguration config{};
  m_candle.ConfigAllSettings(config);
}

void LightsSubsystem::SetRainbowBarf() {
  RainbowAnimation rb = RainbowAnimation(1, 1, m_numLeds);
  m_candle.Animate(rb);
}

// Note that all member functions using "SetLEDs" is setting for the max number of LEDs

void LightsSubsystem::SetCaterpillarYellow() {
  //TODO find out what color this is
  m_candle.SetLEDs(255, 255, 255);
  std::cout << "You haven't implemented LightsSubsystem::SetCaterpillarYellow yet\n";
}

void LightsSubsystem::SetRed() {
  m_candle.SetLEDs(255, 0, 0);
}

void LightsSubsystem::SetGreen() {
  m_candle.SetLEDs(0, 255, 0);
}

void LightsSubsystem::setOrange() {
  m_candle.SetLEDs(255, 111, 0);
}

void LightsSubsystem::BlinkGreen(double speed) {
  StrobeAnimation sa = StrobeAnimation(0, 127, 0, 0, speed);
  m_candle.Animate(sa);
}

void LightsSubsystem::BlinkRed(double speed) {
  StrobeAnimation sa = StrobeAnimation(150, 0, 0, 0, speed);
  m_candle.Animate(sa);
}

// This method will be called once per scheduler run
void LightsSubsystem::Periodic() {}
