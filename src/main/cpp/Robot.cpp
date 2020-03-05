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

//CONFIGURATION
//  Comment out for Tank Drive
#define arcadeDrive
//  Comment out to Disable Auto Ball Pickup
//#define autoBallPickup
//  Uncomment to control Auto Shooting with Machine Learning rather than the Limelight.
//#define portML
//  Uncomment to run Setpoint Configuration
//#define TestSetpoints
string pointTest = "hood";

//CONSTANTS & OTHER GLOBAL VARIABLES

//FUNCTIONAL
//Power of Intake
#define intakePower 1
//Speed of conveyor
#define conveyorSpeed 0.5
//Limelight
#define limelightHeight 584.2
#define limelightY 240
#define focalLength 2.9272781257541
#define outerPortH 762
#define innerPortDepth 742.95

//UNTESTED/UNFUNCTIONAL
//Lift Power
#define liftPower 0.5
//Hood Properties
#define countRange 1680
#define angleRange 15
//Rpm of Shooter
double shooterRPM;
double targetDistance;
double vFeetPerSecond;

//PID
//Elevator PID
#define elevUp 0
#define elevDown 0
#define elevIntermediate 0
#define elevKp 0
#define elevKi 0
double elevSetpoint;
double elevPosition;
//Hood PID
#define hoodDown 0
#define hoodKp 0
#define hoodKi 0
double hoodAngle;
double hoodSetpoint;
double hoodPosition;
//Turret PID
#define turretKp 0
#define turretKi 0
#define minOffset 10
double turretPosition;
double horizontalOffset;
//Elevator
double elevatorPosition;
//Turret Soft Stop + Hood Soft Stop
double turretInit;
double hoodInit;
double elevatorInit;
#define turretMax 105
#define hoodMax -38
//  State Tracking Variables
//Is the shooter in use?
bool shooting = 0;
//Number of Powercells
int powercells;
//Starting Mode of Solenoid
bool solUp = 1;
bool bsolUp = 1;
//Powercell Pickup mode (0 for manual, 1 for auto)
bool pickupMode = 0;


int LEDPWM;


//MOTORS

//Falcon Motor Controller Declaration
TalonSRX leftFrontFalcon = {0};
TalonSRX leftBackFalcon = {1};
TalonSRX rightFrontFalcon = {2};
TalonSRX rightBackFalcon = {3};
//Shooter
// Right should be negative and left should be positive
TalonSRX l_shooter = {4};
TalonSRX r_shooter = {5};
//MCGintake
TalonSRX MCGintake = {6};
//Skywalker
TalonSRX skywalker = {7};

//SparkMax Motor Declaration
//Color Wheel Motor
rev::CANSparkMax colorWheelMotor { 8 , rev::CANSparkMax::MotorType::kBrushed};
//Lift moves powercells from belt to turret
rev::CANSparkMax lift1 { 9 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax lift2 { 10 , rev::CANSparkMax::MotorType::kBrushless};
//Elevator
rev::CANSparkMax elevator { 11 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder elevatorPoint = elevator.GetEncoder();
//Rotation of Shooter
rev::CANSparkMax turret { 12 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder turretPoint = turret.GetEncoder();
//Hood controls angle of Shooter
rev::CANSparkMax hood { 13 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder hoodPoint = hood.GetEncoder();
//Conveyor Belt and Lift
rev::CANSparkMax belt { 14 , rev::CANSparkMax::MotorType::kBrushless};

//Solenoids
frc::Compressor compressor { 0 };
//One of these is up and the other is down ?
frc::Solenoid intakeSolOpen { 0 };
frc::Solenoid intakeSolClose { 1 };
frc::Solenoid brakeSolOff { 4 };
frc::Solenoid colorSolUp { 3 };
frc::Solenoid brakeSolOn { 2 };
frc::Solenoid colorSolDown { 5 };
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

//Machine Learning Data Table
auto inst = nt::NetworkTableInstance::GetDefault();
auto table = inst.GetTable("ML");
//Limelight Data Table
auto llinst = nt::NetworkTableInstance::GetDefault();
auto lltable = llinst.GetTable("limelight");

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

//Cotangent
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
  /*if(horizontalOffset < minOffset){
    l_shooter.Set(ControlMode::Velocity, rpm * 4096 / 600);
    r_shooter.Set(ControlMode::Velocity, rpm * 4096 / 600);

    //load more balls
    lift1.Set(liftPower);
    lift2.Set(-liftPower);
    belt.Set(conveyorSpeed);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "ACTIVE");
  }*/
  //No Velocity control
  l_shooter.Set(ControlMode::PercentOutput, -rpm);
  r_shooter.Set(ControlMode::PercentOutput, rpm);
}
//Intake motors
void intake(double power){
  MCGintake.Set(ControlMode::PercentOutput, power);
}
//PID
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
  //home turret encoder
  turretInit = turretPoint.GetPosition();
  hoodInit = hoodPoint.GetPosition();
  elevatorInit = elevatorPoint.GetPosition();
  //Turn of Limelight LED
  lltable->PutNumber("ledMode", 1);

  //Shooter encoder reset
  
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
void shoot(double power) {
    l_shooter.Set(ControlMode::PercentOutput, power);
    r_shooter.Set(ControlMode::PercentOutput, -power);
}
void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
  //NetworkTable Data
  if(logicontroller.GetRawButton(1)){
    shoot(1);
  } else {
    shoot(0);
  }
  if(logicontroller.GetRawButton(7)){
    belt.Set(conveyorSpeed);
    lift1.Set(liftPower);
    lift2.Set(-liftPower);   
  }else if(logicontroller.GetRawButton(5)){
    belt.Set(-conveyorSpeed);
    lift1.Set(-liftPower);
    lift2.Set(liftPower);
  }else {
    belt.Set(0);
    lift1.Set(0);
    lift2.Set(0);
  }
  
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
  //Input of a button moves color wheel motor until the color of that button is detected.
  /*
  if(logicontroller.GetRawButton(9)) {
    while(colorString != "Blue"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  if(logicontroller.GetRawButton(2)) {
    while(colorString != "Green"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  if(logicontroller.GetRawButton(3)) {
    while(colorString != "Red"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  if(logicontroller.GetRawButton(4)) {
    while(colorString != "Yellow"){
      colorWheelMotor.Set(0.5);
      if(logicontroller.GetRawButton(9)){
        break;
      }
    }
  }
  */
  //Display Number of Powercells
  /*powercells = sensorZero.Get()+sensorOne.Get()+sensorTwo.Get()+sensorThree.Get();
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
  }*/


  //MOVEMENT

  //The if statement below controls the wheels of the robot.
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
  //Intake control MANUAL BACKUP
  if (logicontroller.GetRawButton(6)){
    intake(-intakePower);
  }
  else if (logicontroller.GetRawButton(8)){
    intake(intakePower);
  }
  else {
    intake(0);
  }

//Declaring Variables to store Table Information
  auto boxes = table->GetNumberArray("boxes", {});
  auto object_classes = table->GetStringArray("object_classes", {});

  //Shooter Control
  //Getting Distance
  #ifdef portML
  //USING MACHINE LEARNING
  for (int i; i < boxes.size; i+=4){
    if(object_classes[i] == "outer_port"){
      targetDistance = ((focalLength*outerPortH*limelightY)/((boxes[i+1]-boxes[i+3])*limelightHeight))+innerPortDepth;
    }
  }
  #else
  //Using Limelight
  double llPortH = lltable->GetNumber("tvert", 0)*2;
  targetDistance = 304.8*((focalLength*outerPortH*limelightY)/((llPortH)*limelightHeight))+innerPortDepth;
  horizontalOffset = lltable->GetNumber("tx", 0);
  #endif
  //Getting RPM
  hoodAngle = atan((2*6.52)/targetDistance);
  vFeetPerSecond = (2*(sqrt((6.52*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
  shooterRPM = (vFeetPerSecond*(60*12))/(4*M_PI);
  //Convert Angle to Encoder Counts
  //1680 Counts ~ 15 Degrees
  hoodSetpoint = hoodAngle * countRange / angleRange;
  //Update Encoders
  hoodPosition = hoodPoint.GetPosition();
  hoodPosition = hoodPosition + hoodInit;
  elevatorPosition = elevatorPoint.GetPosition();
  elevatorPosition = elevatorPosition + elevatorInit;
  turretPosition = turretPoint.GetPosition();
  turretPosition = turretPosition - turretInit;
  //Shooter Launch
  /*if(logicontroller.GetRawButton(7)){
    //shooter(shooterRPM);
    shooter(1);
    shooting = 1;
    lltable->PutNumber("ledMode", 3);
    //Hood Up
    hood.Set(PID(hoodSetpoint-hoodPosition, hoodKp, hoodKi));
    //Turret Control
    turret.Set(PID(horizontalOffset, turretKp, turretKi));
  }
  else {
    shooter(0);
    shooting = 0;
    lltable->PutNumber("ledMode", 1);
    //Hood Down
    hood.Set(PID(hoodDown-hoodPosition, hoodKp, hoodKi));
  }
  */

  //Intake Solenoid
  if(logicontroller.GetRawButton(6) && solUp == 0){
    solUp = 1;
    intakeSolOpen.Set(false);
    intakeSolClose.Set(true);
  }
  else if(logicontroller.GetRawButton(8) && solUp == 1){
    solUp = 0;
    intakeSolOpen.Set(true);
    intakeSolClose.Set(false);
  }

  if(r_stick.GetRawButton(3)){
    brakeSolOff.Set(true);
    brakeSolOn.Set(false);
  } else {
    brakeSolOff.Set(false);
    brakeSolOn.Set(true);
  }
  if(r_stick.GetRawButton(2)){
    colorSolUp.Set(true);
    colorSolDown.Set(false);
  } else {
    colorSolUp.Set(false);
    colorSolDown.Set(true);
  }
//Elevator control
///UNTESTED
///NEEDS Ki, Kp
/*elevPosition = elevPoint.GetPosition();
elevator.Set(PID(elevSetpoint - elevPosition, elevKp, elevKi));
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
  //Soft stop for Elevator
  /* NEEDS elevUp VALUE
  if (elevPosition > elevUp || elevPosition < elevDown){
    elevator.Set(0);
  }
  */
 frc::SmartDashboard::PutNumber("Turret Encoder", turretPosition);
 frc::SmartDashboard::PutNumber("Hood Encoder", hoodPosition);
  frc::SmartDashboard::PutNumber("Elevator Encoder", elevatorPosition);
  //Soft stop for Turret
  // NEEDS turretMax VALUE
  if(hoodPosition >= 0){
    hood.Set(-0.2);
  } else if (hoodPosition <= hoodMax){
    hood.Set(0.2);
  } else {
    hood.Set(logicontroller.GetRawAxis(1));
  }
  if(turretPosition >= turretMax){
    turret.Set(-0.05);
  } else if (turretPosition <= -turretMax){
    turret.Set(0.05);
  } else {
    turret.Set(logicontroller.GetRawAxis(0));
  }
  skywalker.Set(ControlMode::PercentOutput, logicontroller.GetRawAxis(2)); 
  elevator.Set(logicontroller.GetRawAxis(3));

#ifdef autoBallPickup
  //Auto Ball Pickup
  ///UNTESTED
  ///INCOMPLETE
  
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

  //  Auto Ball Pickup
  if(l_stick.GetRawButton(2) && pickupMode == 0){
    pickupMode = 1;
  }
  else if(l_stick.GetRawButton(2) && pickupMode == 1){
    pickupMode = 0;
  }

  /* don't work
  for (auto i = 0; i < object_classes.size(); i++) {
    if (object_classes[i] == "powercell" && pickupMode == 1) {
      auto firstBallCoordinate = objectCoordinates[i];
      auto firstBallX = firstBallCoordinate.first;
      PID(firstBallX-640, );
    }
  }
  */
  //???
  for (auto i = 0; i < object_classes.size(); i++) {
    if (object_classes[i] == "powercell") {
      auto firstBallCoordinate = objectCoordinates[i];
      auto firstBallX = firstBallCoordinate.first;
      auto firstBallY = firstBallCoordinate.second;
      break;
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