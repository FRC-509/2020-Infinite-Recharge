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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <ctre/Phoenix.h>
#include <frc/util/color.h> 
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxDriver.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <frc/Servo.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <frc/DigitalInput.h>

/*
wpi::SmallString<64> deployDirectory;
frc::filesystem::GetDeployDirectory(deployDirectory);
wpi::sys::path::append(deployDirectory, "paths");
wpi::sys::path::append(deployDirectory, "AutoONe.wpilib.json");
*/
//frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);


//GLOBAL VARIABLES

#define intakePower 0.5
#define shooterPower 1

int LEDPWM;

//Speed of conveyor(maybe necessary)
#define conveyorSpeed 0.2
//set to 0 for Tank Drive, 1 for Arcade Drive.
#define driveMode 1


//MOTORS

//Falcon Motor Controller Declaration
TalonSRX leftFrontFalcon = {0};
TalonSRX leftBackFalcon = {1};
TalonSRX rightFrontFalcon = {2};
TalonSRX rightBackFalcon = {3};
//Shooter
TalonSRX l_shooter = {4};
TalonSRX r_shooter = {5};
//Color wheel motor
TalonSRX colorWheelMotor = {6};

//SparkMax Motor Declaration
rev::CANSparkMax turret { 4 , rev::CANSparkMax::MotorType::kBrushless};
//Conveyor Belt and Lift
rev::CANSparkMax belt { 9 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax lift1 { 1 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax lift2 { 2 , rev::CANSparkMax::MotorType::kBrushless};
//McGintake
rev::CANSparkMax MCGintakeLeft {6, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax MCGintakeRight {7, rev::CANSparkMax::MotorType::kBrushless};
//Elevator
rev::CANSparkMax elevator { 5 , rev::CANSparkMax::MotorType::kBrushless};

//Servo motor controls rotation of Limelight
frc::Servo servo {0};

frc::DigitalInput sensorZero{0};
frc::DigitalInput sensorOne{1};
frc::DigitalInput sensorTwo{2};
frc::DigitalInput sensThree{3};
//CONTROLLERS

//Logitech Joystick Declaration
frc::Joystick r_stick  {0};
frc::Joystick l_stick  {1};
frc::Joystick logicontroller {2};


//MISC DECLARATIONS

//Set up color sensor
static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
rev::ColorSensorV3 m_colorSensor{i2cPort};
rev::ColorMatch m_colorMatcher;
//Set target color RGB values
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
  //UNTESTED Velocity control
  //l_shooter.Set(ControlMode::Velocity, -power * rpm * 4096 / 600);
  //r_shooter.Set(ControlMode::Velocity, power * rpm * 4096 / 600);

  l_shooter.Set(ControlMode::PercentOutput, -power);
  r_shooter.Set(ControlMode::PercentOutput, power);
}
//Intake motors
void intake(double power){
  MCGintakeLeft.Set(power);
  MCGintakeRight.Set(-power);
}
//Conveyor belt and Lift
void conveyor(double power){
    belt.Set(power);
    lift1.Set(power);
    lift2.Set(-power);
}

//Upon robot startup
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
 
  //Shooter current limiting

  /*l_shooter->EnableCurrentLimit(true); 
  l_shooter->ConfigContinuousCurrentLimit(40,15);
  l_shooter->ConfigPeakCurrentLimit(0,15);

  r_shooter->EnableCurrentLimit(true);
  r_shooter->ConfigContinuousCurrentLimit(40,15);
  r_shooter->ConfigPeakCurrentLimit(0,15);
  */
  //Set motors off initially
  leftFrontFalcon.Set(ControlMode::PercentOutput, 0);
  leftBackFalcon.Set(ControlMode::PercentOutput, 0);
  rightFrontFalcon.Set(ControlMode::PercentOutput, 0);
  rightBackFalcon.Set(ControlMode::PercentOutput, 0);
  l_shooter.Set(ControlMode::PercentOutput, 0);
  r_shooter.Set(ControlMode::PercentOutput, 0);
  colorWheelMotor.Set(ControlMode::PercentOutput, 0);
  turret.Set(0);
  belt.Set(0);
  lift1.Set(0);
  lift2.Set(0);
  MCGintakeLeft.Set(0);
  MCGintakeRight.Set(0);
  
  //Add colors to color match
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
  frc::SmartDashboard::PutNumber("led pwm val", LEDPWM);
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
wpi::SmallString<64> deployDirectory;
frc::filesystem::GetDeployDirectory(deployDirectory);
wpi::sys::path::append(deployDirectory, "paths");
wpi::sys::path::append(deployDirectory, "YourPath.wpilib.json");
frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
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
  //COLOR SENSOR
  if (sensorZero.Get() == true){
    frc::SmartDashboard::PutBoolean("Ball Intake", sensorZero.Get());

  }

  //Color Sensor calculations
  frc::Color detectedColor = m_colorSensor.GetColor();
  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
  //Set colorString to match detected color
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
  //Display color data on SmartDashboard
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
  

  //MOVEMENT

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


  //MISC CONTROLS

  //Conveyor belt control
  if(logicontroller.GetRawButton(5)){
    conveyor(conveyorSpeed);
  }
  //extake
  else if (logicontroller.GetRawButton(7)){
    conveyor(-conveyorSpeed);
  }
  else {
    conveyor(0);
  }
  //Turret rotation control
  turret.Set(logicontroller.GetZ());

  //Servo control
  /*if (){
    LEDPWM = LEDPWM + 1;
    servo.SetAngle(LEDPWM);
  } else if(logicontroller.GetRawButton(6)){
    LEDPWM = LEDPWM - 1;
    servo.SetAngle(LEDPWM);    
  }
  if (logicontroller.GetRawButton(6)){
    servo.SetAngle(105);
  }
  if (logicontroller.GetRawButton(7)){
    servo.SetAngle(75);
  }
  if (logicontroller.GetRawButton(8)){
    servo.SetAngle(60);
  }
  */
  
  //Shooter control
  if(logicontroller.GetRawButton(10)){
    shooter(shooterPower);
  }
  else {
    shooter(0);
  }

  //Intake control
  if (logicontroller.GetRawButton(6)){
    intake(intakePower);
  }
  else if (logicontroller.GetRawButton(8)){
    intake(-intakePower);
  }
  else {
    intake(0);
  }

  //Elevator control
  if(r_stick.GetRawButton(9)){
    //Launch Elevator
    
  }
  else if(r_stick.GetRawButton(10)){
    //Retract Elevator

  }
  else{
    elevator.Set(0);
  }

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

