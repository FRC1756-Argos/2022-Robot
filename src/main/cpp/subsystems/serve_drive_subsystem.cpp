// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/swerve_drive_subsystem.h"
#include "Constants.h"

SwerveDriveSubsystem::SwerveDriveSubsystem()
: m_frontLeft(address::motor::frontLeftDrive, address::motor::frontLeftTurn, address::encoders::frontLeftEncoder)
, m_frontRight(address::motor::frontRightDrive, address::motor::frontRightTurn, address::encoders::frontRightEncoder)
, m_backRight(address::motor::backRightDrive, address::motor::backRightTurn, address::encoders::backRightEncoder)
,m_backLeft(address::motor::backLeftDrive, address::motor::backLeftTurn, address::encoders::backLeftEncoder){

  //create our translation objects
  frc::Translation2d frontLeftCenterOffset(-1 * measureUp::chassis::width / 2 + measureUp::swerveOffsets::frontLeftWOffset,
                                            measureUp::chassis::length - measureUp::swerveOffsets::frontLeftLOffset);

}

// This method will be called once per scheduler run
void SwerveDriveSubsystem::Periodic() {

}

//SEWRVE DRIVE SUBSYSTEM MEMBER FUNCTIONS
void SwerveDriveSubsystem::m_drive(){


}

//SWERVE MODULE SUBSYSTEM FUNCTIONS
SwerveModule::SwerveModule(const char driveAddr, const char turnAddr, const char encoderAddr)
//TODO add constants to file
: m_drive(driveAddr)
, m_turn(turnAddr)
, m_encoder(encoderAddr){

}
