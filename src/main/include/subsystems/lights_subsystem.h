// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"

class LightsSubsystem : public frc2::SubsystemBase {
 public:
  LightsSubsystem() = delete;

  // addr -> Address of the CANdle
  // canBus -> Can bus to send CAN traffic to, defaults to built-in rio bus
  LightsSubsystem(u_int numLeds, u_char addr, std::string canBus = "rio");

  // Sets the robot's LEDs to rainbow
  void SetRainbowBarf();

  // Sets the robot's LEDs to yellow
  void SetCaterpillarYellow();

  // you guessed it, it makes the LEDs red
  void SetRed();

  // makes the LEDs green
  void SetGreen();

  // Sets LEDs to orange
  void setOrange();

  // Blinks the LEDs green
  // speed -> the speed of the strobe animation. Defaults to 0.5
  void BlinkGreen(double speed = 0.5);

  // Blinks the LEDs red
  // speed -> the speed of the strobe animation. Defaults to 0.5
  void BlinkRed(double speed = 0.5);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  CANdle m_candle;
  u_int m_numLeds;
};
