/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//LIBRARIES

#include "Robot.h"
#include <frc/Joystick.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxDriver.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <frc/Servo.h>


//GLOBAL VARIABLES

#define shooterPower 1


//MOTORS

//Falcon Motor Controller Declaration
TalonSRX leftFrontFalcon = {0};
TalonSRX leftBackFalcon = {1};
TalonSRX rightFrontFalcon = {2};
TalonSRX rightBackFalcon = {3};

//Shooter
TalonSRX l_shooter = {6};
TalonSRX r_shooter = {7};

//Color wheel motor
TalonSRX colorWheelMotor = {8};

//SparkMax Motor Declaration
rev::CANSparkMax turret { 4 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax conveyor { 9 , rev::CANSparkMax::MotorType::kBrushless};

//Servo motor controls rotation of Limelight
frc::Servo servo {0};


//CONTROLLERS

//Logitech Joystick Declaration
frc::Joystick r_stick  {0};
frc::Joystick l_stick  {1};
frc::Joystick logicontroller {2};


//MISC DECLARATIONS

//set to 0 for Tank Drive, 1 for Arcade Drive.
bool driveMode = 1;

static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
rev::ColorSensorV3 m_colorSensor{i2cPort};
rev::ColorMatch m_colorMatcher;
static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);


//FUNCTIONS

//Sync left wheel motors
void leftDrive(double power){
  leftFrontFalcon.Set(ControlMode::PercentOutput, power);
  leftBackFalcon.Set(ControlMode::PercentOutput, power);
}
//Sync right wheel motors
void rightDrive(double power){
  rightFrontFalcon.Set(ControlMode::PercentOutput, -power);
  rightBackFalcon.Set(ControlMode::PercentOutput, -power);
}
//Shooter motors
void shooter(double power){
  l_shooter.Set(ControlMode::PercentOutput, power);
  r_shooter.Set(ControlMode::PercentOutput, -power);
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Set motors off initially
  leftFrontFalcon.Set(ControlMode::PercentOutput, 0);
  leftBackFalcon.Set(ControlMode::PercentOutput, 0);
  rightFrontFalcon.Set(ControlMode::PercentOutput, 0);
  rightBackFalcon.Set(ControlMode::PercentOutput, 0);

  //Set Neo 550's to off
  turret.Set(0);
  conveyor.Set(0);

  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {
  
  //The if statement below controls the wheels of the robot.
  if(driveMode == 1){
    //Arcade Drive
    leftDrive(l_stick.GetY() - (0.5 * (r_stick.GetX())));
    rightDrive(l_stick.GetY() + (0.5 *(r_stick.GetX())));
  }
  else if(driveMode == 0){
    //Tank Drive
    leftDrive(l_stick.GetY());
    rightDrive(r_stick.GetY());
  }
  else {
    //cout << "driveMode boolean is somehow neither 1 nor 0.";
  }

  frc::Color detectedColor = m_colorSensor.GetColor();
  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

  if (matchedColor == kBlueTarget) {
    colorString = "Blue";
  }
  else if (matchedColor == kGreenTarget) {
    colorString = "Green";
  }
  else if (matchedColor == kRedTarget) {
    colorString = "Red";
  }
  else if (matchedColor == kYellowTarget) {
    colorString = "Yellow";
  }
  else {
    colorString = "Unknown";
  }

  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Color", colorString);
 
  //Input of a button moves color wheel motor until that button is detected.
  if(logicontroller.GetRawButton(1)) {
    while(colorString != "Blue"){
      colorWheelMotor.Set(ControlMode::PercentOutput, 0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  if(logicontroller.GetRawButton(2)) {
    while(colorString != "Green"){
      colorWheelMotor.Set(ControlMode::PercentOutput, 0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  if(logicontroller.GetRawButton(3)) {
    while(colorString != "Red"){
      colorWheelMotor.Set(ControlMode::PercentOutput, 0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  if(logicontroller.GetRawButton(4)) {
    while(colorString != "Yellow"){
      colorWheelMotor.Set(ControlMode::PercentOutput, 0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  
  //Conveyor belt control
  if(logicontroller.GetRawButton(5)){
    conveyor.Set(0.2);
  }
  else if (logicontroller.GetRawButton(7)){
    conveyor.Set(-0.2);
  }
  else {
    conveyor.Set(0);
  }

  //Turret control
  turret.Set(logicontroller.GetZ());

  //Servo control
  if (l_stick.GetRawButton(6)){
    servo.SetAngle(120);
  }
  if (l_stick.GetRawButton(7)){
    servo.SetAngle(60);
  }

  //Shooter control
  if(logicontroller.GetRawButton(8)){
    shooter(shooterPower);
  }
  else {
    shooter(0);
  }

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
