/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//LIBRARIES

#include "Robot.h"
#include <iostream>
#include <cmath>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/color.h>
#include <frc/Servo.h>
#include <frc/Filesystem.h>
#include <frc/Encoder.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/DigitalInput.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxDriver.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/Phoenix.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <wpi/Path.h>
#include <wpi/SmallString.h>

using namespace std;


//GLOBAL VARIABLES


//  CONFIGURATION
//    Comment out for Tank Drive
#define arcadeDrive
//    Comment out to Disable Auto Ball Pickup
#define autoBallPickup
//    Uncomment to control Auto Shooting with Machine Learning rather than the Limelight.
//#define portML
//    Uncomment to run Setpoint Configuration
//#define TestSetpoints
string pointTest = "";


//CONSTANTS & OTHER GLOBAL VARIABLES

//  FUNCTIONAL
//    Speed of Intake
#define intakePower 0.5
//    Speed of conveyor
#define conveyorSpeed 0.5
//    Speed of Lift
#define liftPower 0.5
//    Limelight
#define limelightHeight 584.2
#define limelightY 240
#define focalLength 2.9272781257541
#define outerPortH 762
#define innerPortDepth 742.95

//  UNTESTED/UNFUNCTIONAL
//    Hood Properties
#define countRange 1680
#define angleRange 15


//  PID
//    Elevator PID
//    *NOT TUNED
#define elevUp 0
#define elevDown 0
#define elevIntermediate 0
#define elevKp 0
#define elevKi 0
#define elevMaxVelocity 0.3
#define elevMinVelocity 0.05
double elevSetpoint;
double elevPosition;
double elevVelocity;
//    Hood PID
//    *NOT TUNED
#define hoodDown 0
#define hoodKp 0
#define hoodKi 0
double hoodAngle;
double hoodSetpoint;
double hoodPosition;
//    Turret PID
//    *NOT TUNED
#define turretKp 0
#define turretKi 0
#define minOffset 10
double turretPosition;
double horizontalOffset;
//    Ball Pickup PID
#define pickupKp 0
#define pickupKi 0
//Turret Soft Stop
#define turretMax 0

//  STATE TRACKING VARIABLES
//    Is the shooter in use?
bool shooting = 0;
//    Number of Powercells in conveyor
int powercells = 0;
//    Starting Mode of Solenoids
bool intakeSolUp = 1;
bool colorWheelSolUp = 1;
bool elevBrakeSolUp = 1;
//    Powercell Pickup mode (0 for manual, 1 for auto)
bool pickupMode = 0;
//    Shooter Calculations
double shooterRPM;
double targetDistance;
double vFeetPerSecond;


//?
int LEDPWM;


//MOTORS

//Falcon Motor Controller Declaration
TalonSRX leftFrontFalcon = {0};
TalonSRX leftBackFalcon = {1};
TalonSRX rightFrontFalcon = {2};
TalonSRX rightBackFalcon = {3};
//Shooter
TalonSRX l_shooter = {4};
TalonSRX r_shooter = {5};
//MCGintake
TalonSRX MCGintake = {6};
//Skywalker
TalonSRX skywalker = {7};

//SparkMax Motor/Encoder Declaration
//Color Wheel Motor
rev::CANSparkMax colorWheelMotor { 8 , rev::CANSparkMax::MotorType::kBrushed};
//Lift moves powercells from belt to turret
rev::CANSparkMax lift1 { 9 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax lift2 { 10 , rev::CANSparkMax::MotorType::kBrushless};
//Elevator
rev::CANSparkMax elevator { 11 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder elevPoint = elevator.GetEncoder();
//Rotation of Shooter
rev::CANSparkMax turret { 12 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder turretPoint = turret.GetEncoder();
//Hood controls angle of Shooter
rev::CANSparkMax hood { 13 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder hoodPoint = hood.GetEncoder();
//Conveyor Belt and Lift
rev::CANSparkMax belt { 14 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder beltEncoder = belt.GetEncoder();

//Solenoids
frc::Compressor compressor { 0 };
//One of these is up and the other is down ?
frc::Solenoid intakeSolOpen { 0 };
frc::Solenoid intakeSolClose { 1 };
frc::Solenoid colorWheelSolOpen { 2 };
frc::Solenoid colorWheelSolClose { 3 };
frc::Solenoid elevBrakeSolOpen { 4 };
frc::Solenoid elevBrakeSolClose { 5 };


//Digital Sensors
/*frc::DigitalInput sensorZero{3};
frc::DigitalInput sensorOne{2};
frc::DigitalInput sensorTwo{1};
frc::DigitalInput sensorThree{0};*/


//CONTROLLERS

frc::Joystick r_stick  {0};
frc::Joystick l_stick  {1};
frc::Joystick logicontroller {2};


//MISC DECLARATIONS

//  NETWORK TABLES
//    Machine Learning Data Table
auto inst = nt::NetworkTableInstance::GetDefault();
auto table = inst.GetTable("ML");
//    Limelight Data Table
auto llinst = nt::NetworkTableInstance::GetDefault();
auto lltable = llinst.GetTable("limelight");
//  COLOR SENSOR
//    Set up color sensor
static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
rev::ColorSensorV3 m_colorSensor{i2cPort};
rev::ColorMatch m_colorMatcher;
//    Set target color RGB values
static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);


//FUNCTIONS

//  Color Wheel's Function in TeleopPeriodic was long, so it was moved here to be condensed in VSCODE.
void colorWheel(){
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
  //Input of a button moves color wheel motor until the color of that button is detected.
   if(logicontroller.GetRawButton(1)) {
    while(colorString != "Blue"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
    else {
      colorWheelMotor.Set(0);
    }
  }
  if(logicontroller.GetRawButton(2)) {
    while(colorString != "Green"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
    else {
      colorWheelMotor.Set(0);
    }
  }
  if(logicontroller.GetRawButton(3)) {
    while(colorString != "Red"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
    else {
      colorWheelMotor.Set(0);
    }
  }
  if(logicontroller.GetRawButton(4)) {
    while(colorString != "Yellow"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
    else {
      colorWheelMotor.Set(0);
    }
  }
}
//  Cotangent
double cotan(double i){
  return 1/tan(i);
}
//  Sync left wheel motors
void leftDrive(double power){
  leftFrontFalcon.Set(ControlMode::PercentOutput, power);
  leftBackFalcon.Set(ControlMode::PercentOutput, power);
}
//  Sync right wheel motors
void rightDrive(double power){
  rightFrontFalcon.Set(ControlMode::PercentOutput, -power);
  rightBackFalcon.Set(ControlMode::PercentOutput, -power);
}
//  Shooter motors
void shooter(double rpm){
  //UNTESTED Velocity control
  //ControlMode::Velocity looks for Units/100ms
  //Convert Revolutions/Minute -> Units per 100ms
  //Revolutions/Minute * Units/Revolution (4096) = Units/Minute
  //Units/Minute * 1/600 = Units/100ms
  if(horizontalOffset < minOffset && shooting == 1){
    l_shooter.Set(ControlMode::Velocity, rpm * 4096 / 600);
    r_shooter.Set(ControlMode::Velocity, rpm * 4096 / 600);

    //load more balls
    lift1.Set(liftPower);
    lift2.Set(-liftPower);
    belt.Set(conveyorSpeed);
  }
  //No Velocity control
  //l_shooter.Set(ControlMode::PercentOutput, -power);
  //r_shooter.Set(ControlMode::PercentOutput, power);
}
//  Intake motors
void intake(double power){
  MCGintake.Set(ControlMode::PercentOutput, power);
}
//  PID
double PID(double error, double Kp, double Ki){
  double p;
  double i;
  double integral;

  integral += error;
  p = Kp*error;
  i = Ki*integral;
  return p+i;
}

//Upon robot startup
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
 
  //Turn of Limelight LED
  lltable->PutNumber("ledMode", 1);

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
  skywalker.Set(ControlMode::PercentOutput, 0);
  colorWheelMotor.Set(0);
  turret.Set(0);
  belt.Set(0);
  lift1.Set(0);
  lift2.Set(0);
  MCGintake.Set(ControlMode::PercentOutput, 0);
  
  //Add colors to color match
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("led pwm val", LEDPWM);
}

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

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //SMARTDASHBOARD
  //  NetworkTable Data
  frc::SmartDashboard::PutNumber("Number of Objects", table->GetNumber("nb_objects", 0));
  frc::SmartDashboard::PutStringArray("Object Types", table->GetStringArray("object_classes", {}));
  //  Belt State
  if(beltEncoder.getVelocity() > 0){
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "ACTIVE");
  }
  else if(beltEncoder.getVelocity < 0){
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "REVERSED");
  }
  else {
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  }
  //  Display Number of Powercells
  //    COMMENTED BECAUSE SENSORS ARE NOT WIRED
  /*powercells = sensorZero.Get()+sensorOne.Get()+sensorTwo.Get()+sensorThree.Get();
  frc::SmartDashboard::PutBoolean("zero", sensorZero.Get());
  frc::SmartDashboard::PutBoolean("one", sensorOne.Get());
  frc::SmartDashboard::PutBoolean("two", sensorTwo.Get());
  frc::SmartDashboard::PutBoolean("three", sensorThree.Get());
  frc::SmartDashboard::PutNumber("Powercells", powercells);*/


  //COLOR SENSOR
  colorWheel();


  //AUTO CONVEYOR CONTROL
  /*
  if(sensorZero.Get() == 1){
    belt.Set(conveyorSpeed);
  }
  if(sensorZero.Get() == 0 && shooting == 0) {
    belt.Set(0);
  }*/


  //MOVEMENT
  #ifdef arcadeDrive
    //Arcade Drive
    leftDrive(l_stick.GetY() - (0.5 * (r_stick.GetX())));
    rightDrive(l_stick.GetY() + (0.5 *(r_stick.GetX())));
  #else
    //Tank Drive
    leftDrive(l_stick.GetY());
    rightDrive(r_stick.GetY());
  #endif


  //MISC CONTROLS

  //  MANUAL BACKUPS
  //    Conveyor belt
  if(l_stick.GetRawButton(6)){ 
    belt.Set(conveyorSpeed);
  }
  else if(l_stick.GetRawButton(7)){
    belt.Set(-conveyorSpeed);
  }
  else if(l_stick.GetRawButton(8)){
    belt.Set(0);
  }
  //    Intake
  if (logicontroller.GetRawButton(6)){
    //Intake
    intake(-intakePower);
  }
  else if (logicontroller.GetRawButton(8)){
    //Extake
    intake(intakePower);
  }
  else {
    intake(0);
  }

  //Updating Machine Learning Network Table
  auto boxes = table->GetNumberArray("boxes", {});
  auto object_classes = table->GetStringArray("object_classes", {});

  //SHOOTER

  //  CALCULATIONS
  //    Calculating Distance
  #ifdef portML
  //      Via Machine Learning
  for (int i; i < boxes.size; i+=4){
    if(object_classes[i] == "outer_port"){
      targetDistance = ((focalLength*outerPortH*limelightY)/((boxes[i+1]-boxes[i+3])*limelightHeight))+innerPortDepth;
    }
  }
  #else
  //      Via Limelight
  double llPortH = lltable->GetNumber("tvert", 0)*2;
  targetDistance = 304.8*((focalLength*outerPortH*limelightY)/((llPortH)*limelightHeight))+innerPortDepth;
  horizontalOffset = lltable->GetNumber("tx", 0);
  #endif

  //    Converting Distance to RPM
  hoodAngle = atan((2*6.52)/targetDistance);
  vFeetPerSecond = (2*(sqrt((6.52*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
  shooterRPM = (vFeetPerSecond*(60*12))/(4*M_PI);

  //    Converting Angle to Encoder Counts
  //      1680 Counts ~ 15 Degrees
  hoodSetpoint = hoodAngle * countRange / angleRange;

  //  Update Encoders
  hoodPosition = hoodPoint.GetPosition();
  turretPosition = turretPoint.GetPosition();

  //  Shooter Launch
  if(logicontroller.GetRawButton(7)){
    shooter(shooterRPM);
    shooting = 1;
    lltable->PutNumber("ledMode", 3);
    //Turret Control
    turret.Set(PID(horizontalOffset, turretKp, turretKi));
  }
  else {
    shooter(0);
    shooting = 0;
    lltable->PutNumber("ledMode", 1);
  }

  if(PID(hoodSetpoint-hoodPosition, hoodKp, hoodKi) > 0.25){
    //Hood Up Capped Speed
    hood.Set(0.25);
  }
  else if(PID(hoodDown-hoodPosition, hoodKp, hoodKi) < -0.25){
    //Hood Down Capped Speed
    hood.Set(-0.25);
  }
  else if (shooting == 1){
    //Hood Up PID Controlled
    hood.Set(PID(hoodSetpoint-hoodPosition, hoodKp, hoodKi));
  }
  else if (shooting == 0){
    //Hood Down PID Controlled
    hood.Set(PID(hoodDown-hoodPosition, hoodKp, hoodKi));
  }


  //Elevator control
  ///UNTESTED
  ///NEEDS Ki, Kp

  //  Velocity Control
  elevVelocity = elevPoint.GetVelocity();
  elevPosition = elevPoint.GetPosition();
  if(PID(elevSetpoint - elevPosition, elevKp, elevKi) > elevMaxVelocity){
    //Elevator Moves at MAX VELOCITY +
    elevator.Set(elevMaxVelocity);
    elevBrakeSolOpen.Set(false);
    elevBrakeSolClose.Set(true);
  }
  else if(PID(elevSetpoint - elevPosition, elevKp, elevKi) < elevMaxVelocity){
    //Elevator Moves at MAX VELOCITY -
    elevator.Set(-elevMaxVelocity);
    elevBrakeSolOpen.Set(false);
    elevBrakeSolClose.Set(true);
  }
  else if(abs(PID(elevSetpoint - elevPosition, elevKp, elevKi)) < elevMinVelocity){
    //Elevator STOPS
    elevator.Set(0);
    elevBrakeSolOpen.Set(true);
    elevBrakeSolClose.Set(false);
  }
  else {
    //Elevator Moves at PID CONTROLLED VELOCITY
    elevator.Set(PID(elevSetpoint - elevPosition, elevKp, elevKi));
    elevBrakeSolOpen.Set(false);
    elevBrakeSolClose.Set(true);
  }

  //  PID Setpoint control
  if(r_stick.GetRawButton(10)){
    //Extend Elevator
    elevSetpoint = elevUp;
  }
  else if (r_stick.GetRawButton(11)){
    //Climb
    elevSetpoint = elevIntermediate;
  }
  else if(r_stick.GetRawButton(9)){
    //Retract Elevator
    elevSetpoint = elevDown;
  }


  //SOFT STOPS
  //  Elevator
  /* NEEDS elevUp AND elevDown VALUE
  if (elevPosition > elevUp || elevPosition < elevDown){
    elevator.Set(0);
  }
  */
  //  Turret
  /* NEEDS turretMax VALUE
  / NEEDS turretMax VALUE
  if(turretPoint >= turretMax || turretPoint <= -turretMax){
    turret.Set(0);
  }
  */

  //SOLENOIDS
  //  Intake Solenoid
  if(logicontroller.GetRawButtonPressed(5) && solUp == 0){
    solUp = 1;
    intakeSolOpen.Set(false);
    intakeSolClose.Set(true);
  }
  else if(logicontroller.GetRawButtonPressed(5) && solUp == 1){
    solUp = 0;
    intakeSolOpen.Set(true);
    intakeSolClose.Set(false);
  }
  //  Color Wheel Solenoid
  if(logicontroller.GetRawButtonPressed(9) && colorWheelSolUp == 0){
    colorWheelSolUp = 1;
    colorWheelSolOpen.Set(false);
    colorWheelSolClose.Set(true);
  }
  else if(logicontroller.GetRawButtonPressed(9) && colorWheelSolUp == 1){
    colorWheelSolUp = 0;
    colorWheelSolOpen.Set(true);
    colorWheelSolClose.Set(false);
  }

  //SKYWALKER
  if(l_stick.GetRawButton(4)){
    skywalker.Set(0.2);
  }
  else if(l_stick.GetRawButton(5)){
    skywalker.Set(-0.2);
  }
  else {
    skywalker.Set(0);
  }


#ifdef autoBallPickup
  //Auto Ball Pickup
  ///UNTESTED
  ///INCOMPLETE
  /// MOVE THIS TO AN OUTSIDE FUNCTION WHEN COMPLETE FOR USE IN AUTONOMOUS
  
  //  Variable to store object center coordinate
  vector< pair<double, double> > objectCoordinates = {};
  //  For each object, average the coordinates into a center coordinate. Push this to objectCoordinates.
  for (auto i = 0; i < boxes.size(); i += 4) {
    /* [top_left__x1, top_left_y1, bottom_right_x1, bottom_right_y1, top_left_x2, top_left_y2, … ] */
    auto centerX = (boxes[i+0] + boxes[i+2]) / 2;
    auto centerY = (boxes[i+1] + boxes[i+3]) / 2;
    auto coordinate = make_pair(centerX, centerY);
    objectCoordinates.push_back(coordinate);
  }

  //  Toggle Auto Pickup. Off by Default.
  if(l_stick.GetRawButton(2) && pickupMode == 0){
    pickupMode = 1;
  }
  else if(l_stick.GetRawButton(2) && pickupMode == 1){
    pickupMode = 0;
  }

  //  Drive Toward Ball
  for (auto i = 0; i < object_classes.size(); i++) {
    if (object_classes[i] == "powercell" && pickupMode == 1) {
      auto firstBallCoordinate = objectCoordinates[i];
      auto firstBallX = firstBallCoordinate.first;
      leftDrive(0.2*PID(firstBallX-640, pickupKp, pickupKi));
      rightDrive(-0.2*PID(firstBallX-640, pickupKp, pickupKi));
    }
  }
  #endif

  /*  THE CODE BELOW IS FOR FINDING SETPOINTS FOR PID
      COMMENT IT OUT UNLESS WE NEED TO REDO PID SETPOINTS */
  #ifdef TestSetpoints
  if(pointTest == "hood"){
    cout << hoodPosition;
  }
  else if (pointTest == "elevator"){
    cout << elevPosition;
  }
  else if (pointTest == "turret"){
    cout << turretPosition;
  }
  #endif
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif