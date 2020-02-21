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
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <cmath>

using namespace std;


//GLOBAL VARIABLES

//FUNCTIONAL
//Power of Intake
#define intakePower 0.5
//Speed of conveyor
#define conveyorSpeed 0.5
//set to 0 for Tank Drive, 1 for Arcade Drive.
#define driveMode 1
//Is the shooter in use?
bool shooting = 0;

//UNTESTED/UNFUNCTIONAL
//Lift Power
#define liftPower 0.5
//Rpm of Shooter
double shooterRPM;
double targetDistance;
double hoodAngle;
double vFeetPerSecond;

int LEDPWM;


//MOTORS

//Falcon Motor Controller Declaration
TalonSRX leftFrontFalcon = {0};
TalonSRX leftBackFalcon = {1};
TalonSRX rightFrontFalcon = {2};
TalonSRX rightBackFalcon = {3};
//MCGintake
TalonSRX MCGintake = {7};
//Shooter
TalonSRX l_shooter = {4};
TalonSRX r_shooter = {5};
//Color wheel motor
//TalonSRX colorWheelMotor = {6};

//SparkMax Motor Declaration
rev::CANSparkMax turret { 7 , rev::CANSparkMax::MotorType::kBrushless};
//Conveyor Belt and Lift
rev::CANSparkMax belt { 9 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax lift1 { 4 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax lift2 { 5 , rev::CANSparkMax::MotorType::kBrushless};
//hood
rev::CANSparkMax hood { 8 , rev::CANSparkMax::MotorType::kBrushless};
//Elevator
rev::CANSparkMax elevator { 6 , rev::CANSparkMax::MotorType::kBrushless};

//Solenoids
frc::Compressor compressor { 0 };
//One of these is up and the other is down ?
frc::Solenoid intakeSolOpen { 0 };
frc::Solenoid intakeSolClose { 1 };

//Digital Sensors
frc::DigitalInput sensorZero{3};
frc::DigitalInput sensorOne{2};
frc::DigitalInput sensorTwo{1};
frc::DigitalInput sensorThree{0};


//CONTROLLERS

frc::Joystick r_stick  {0};
frc::Joystick l_stick  {1};
frc::Joystick logicontroller {2};


//MISC DECLARATIONS

//Machine Learning Data Table
auto inst = nt::NetworkTableInstance::GetDefault();
auto table = inst.GetTable("ML");

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

double cotan(double i){
  return 1/tan(i);
}
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
void shooter(double rpm){
  //UNTESTED Velocity control
  //ControlMode::Velocity looks for Units/100ms
  //Convert Revolutions/Minute -> Units per 100ms
  //Revolutions/Minute * Units/Revolution (4096) = Units/Minute
  //Units/Minute * 1/600 = Units/100ms
  //l_shooter.Set(ControlMode::Velocity, rpm * 4096 / 600);
  //r_shooter.Set(ControlMode::Velocity, rpm * 4096 / 600);

  //Kinda works? No Velocity control
  //l_shooter.Set(ControlMode::PercentOutput, -power);
  //r_shooter.Set(ControlMode::PercentOutput, power);
  
  //load more balls
  lift1.Set(liftPower);
  lift2.Set(-liftPower);
  belt.Set(conveyorSpeed);
  frc::SmartDashboard::PutString("CONVEYOR BELT:", "ACTIVE");
}
//Intake motors
void intake(double power){
  MCGintake.Set(ControlMode::PercentOutput, power);
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
  //colorWheelMotor.Set(ControlMode::PercentOutput, 0);
  turret.Set(0);
  belt.Set(0);
  frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  lift1.Set(0);
  lift2.Set(0);
  MCGintake.Set(ControlMode::PercentOutput, 0);
  
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
  cout << "Auto selected: " << m_autoSelected << endl;

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
  //NetworkTable Data
  //Put Data
  frc::SmartDashboard::PutNumber("Number of Objects", table->GetNumber("nb_objects", 0));
  frc::SmartDashboard::PutStringArray("Object Types", table->GetStringArray("object_classes", {}));

  //COLOR SENSOR

  //Color Sensor calculations
  frc::Color detectedColor = m_colorSensor.GetColor();
  string colorString;
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
  /* COMMENTED BECAUSE COLOR WHEEL MOTOR NOT WIRED
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
  */
  //Display Number of Powercells
  int powercells;
  frc::SmartDashboard::PutBoolean("zero", sensorZero.Get());
  frc::SmartDashboard::PutBoolean("one", sensorOne.Get());
  frc::SmartDashboard::PutBoolean("two", sensorTwo.Get());
  frc::SmartDashboard::PutBoolean("three", sensorThree.Get());
  frc::SmartDashboard::PutNumber("Powercells", powercells);

  //Automatic Conveyor Belt Control
  if(sensorZero.Get() == 1){
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "ACTIVE");
    belt.Set(conveyorSpeed);
  }
  if(sensorZero.Get() == 0 && shooting == 0) {
    belt.Set(0);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
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

  //Conveyor belt MANUAL CONTROL BACKUP
  if(l_stick.GetRawButton(6)){ 
    belt.Set(conveyorSpeed);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "ACTIVE");
  }
  else if(l_stick.GetRawButton(7)){
    belt.Set(-conveyorSpeed);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "REVERSE");
  }
  else if(l_stick.GetRawButton(8)){
    belt.Set(0);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  }
  
  //Turret rotation control
  turret.Set(logicontroller.GetZ());
  
  //Shooter control
  if(logicontroller.GetRawButton(7)){
    shooter(shooterRPM);
    shooting = 1;
  } 
  else {
    shooter(0);
    shooting = 0;
  }
  //RPM Calculations
  hoodAngle = atan((2*6.52)/targetDistance);
  vFeetPerSecond = (2*(sqrt((6.52*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
  shooterRPM = targetDistance;

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

  //Intake Solenoid
  bool solUp = 1;
  if(logicontroller.GetRawButton(5) && solUp == 0){
    solUp = 1;
    intakeSolOpen.Set(false);
    intakeSolClose.Set(true);
  }
  else if(logicontroller.GetRawButton(5) && solUp == 1){
    solUp = 0;
    intakeSolOpen.Set(true);
    intakeSolClose.Set(false);
  }

//Elevator control
///INCOMPLETE
///UNTESTED
///REQUIRES PID
  if(r_stick.GetRawButton(10)){
    //Launch Elevator
    
  }
  
  else if(r_stick.GetRawButton(11)){
    //Retract Elevator

  }
  else{
    //elevator.Set(0);
  }

  //Auto Ball Pickup
  ///UNTESTED
  ///INCOMPLETE

  //Declaring Variables to store Table Information
  auto boxes = table->GetNumberArray("boxes", {});
  auto object_classes = table->GetStringArray("object_classes", {});
  
  //Variable to store object center coordinate
  vector< pair<double, double> > objectCoordinates = {};
  //For each object, average the coordinates into a center coordinate. Push this to objectCoordinates.
  for (auto i = 0; i < boxes.size(); i += 4) {
    /* [top_left__x1, top_left_y1, bottom_right_x1, bottom_right_y1, top_left_x2, top_left_y2, … ] */
    auto centerX = (boxes[i+0] + boxes[i+2]) / 2;
    auto centerY = (boxes[i+1] + boxes[i+3]) / 2;
    auto coordinate = make_pair(centerX, centerY);
    objectCoordinates.push_back(coordinate);
  }

  //???
  for (auto i = 0; i < object_classes.size(); i++) {
    if (object_classes[i] == "") {
      auto firstBallCoordinate = objectCoordinates[i];
      auto firstBallX = firstBallCoordinate.first;
      auto firstBallY = firstBallCoordinate.second;
      break;
    }
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
