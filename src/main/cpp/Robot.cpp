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
//  Comment to disable Encoder output in SmartDashboard
#define TestSetpoints
//  Comment out to Disable Auto Ball Pickup
//#define autoBallPickup
//  Uncomment to control Auto aiming with Machine Learning rather than the Limelight.
//#define portML


//CONSTANTS & OTHER GLOBAL VARIABLES

//FUNCTIONAL
//Power of Intake
#define intakePower 0.75
//Speed of conveyor
#define conveyorSpeed 0.25
//Limelight
#define limelightHeight 584.2
#define limelightY 240
#define focalLength 2.9272781257541
#define outerPortH 762
#define innerPortDepth 742.95
//Lift Properties
#define liftPower 0.5
//Hood Properties
#define angleRange M_PI/12
//Rpm of Shooter
double shooterRPM = std::clamp(0, 0, 8000);
double targetDistance;
double vFeetPerSecond;
bool atSpeed = 0;
double realShooterVelocity;
#define shooterAdjustment 1.05
//Elevator
double elevatorPosition;
//Turret Soft Stop + Hood Soft Stop
double turretInit;
double hoodInit;
double elevatorInit;
#define turretMax 105
#define hoodMax -40

//PID
//  Elevator PID
#define elevatorUp 0
#define elevatorDown 0
#define elevatorIntermediate 0
#define elevatorKp 0
#define elevatorKi 0
double elevatorSetPoint;
//  Hood PID
#define hoodDown 0
#define hoodKp 0.005
#define hoodKi 0
double hoodAngle;
double hoodSetpoint;
double hoodPosition;
//  Turret PID
#define turretKp 0.06
#define turretKi 0.005
#define minOffset 10
double turretPosition;
double horizontalOffset;
double shooterInput;

//  State Tracking Variables
//Is the shooter in use?
bool aiming = 0;
//Number of Powercells
int powercells;
bool feed = 0;
//Starting Mode of Solenoid
bool intakeSolUp = 1;
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

//CURRENT LIMITING
WPI_TalonFX * shooterEncoder = new WPI_TalonFX{4};
//ConfigPeakCurrentLimit();

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

//Current Limiting for Neo and Neo 550's

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
frc::DigitalInput sensorIntake{1};
frc::DigitalInput sensorExit{0};


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
//Shooter motors
void shooter(double rpm){
  //UNTESTED Velocity control
  //ControlMode::Velocity looks for Units/100ms
  //Convert Revolutions/Minute -> Units per 100ms
  //Revolutions/Minute * Units/Revolution (4096) = Units/Minute
  frc::SmartDashboard::PutString("SHOOTER:", "ACTIVE");
  frc::SmartDashboard::PutBoolean("at speed?", atSpeed);
  frc::SmartDashboard::PutNumber("realShooterVelocity", realShooterVelocity);
  //Units/Minute * 1/600 = Units/100ms
  
  l_shooter.Set(ControlMode::Velocity, -1 * rpm * 4096/600);
  r_shooter.Set(ControlMode::Velocity, rpm * 4096/600);
  if(horizontalOffset < minOffset){
    frc::SmartDashboard::PutNumber("rpm", rpm);
    if(rpm < 0){
      lift1.Set(liftPower);
      lift2.Set(-liftPower);
      belt.Set(conveyorSpeed);
    }
  }else if(logicontroller.GetRawButton(5)){
      lift1.Set(liftPower);
      lift2.Set(-liftPower);
      belt.Set(conveyorSpeed);
  }  else {
    lift1.Set(0);
    lift2.Set(0);
    //belt.Set(0);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  
  //No Velocity control
  /*l_shooter.Set(ControlMode::PercentOutput, -rpm);
  r_shooter.Set(ControlMode::PercentOutput, rpm);*/
  }
  /*
  if(rpm <= 0){
    lift1.Set(0);
    lift2.Set(0);
    belt.Set(0);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  }
  */
}
//Intake motors
void intake(double power){
  MCGintake.Set(ControlMode::PercentOutput, -power);
}




//Upon robot startup
void Robot::RobotInit() {
  //Current Limiting
  realShooterVelocity = shooterEncoder->GetSelectedSensorVelocity();
  lift1.SetSmartCurrentLimit(10);
  lift2.SetSmartCurrentLimit(10);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //home turret encoder
  turretInit = turretPoint.GetPosition();
  hoodInit = hoodPoint.GetPosition();
  elevatorInit = elevatorPoint.GetPosition();
  //Turn off Limelight LED
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
  //TEMPORARY:
  //  Elevator control
  if(logicontroller.GetRawButton(9)){
    elevator.Set(0.1);
  } else if(logicontroller.GetRawButton(10)){
    elevator.Set(-1);
  } else {
    elevator.Set(logicontroller.GetRawAxis(3));
  }
  //  Shoot

  frc::SmartDashboard::PutBoolean("aiming", aiming);
  if(logicontroller.GetRawButton(3)){
    aiming = 1;
  }
  else if (logicontroller.GetRawButton(1)){
    aiming = 0;
  }
    
  if(logicontroller.GetRawButton(4)){
    //shoot(1);
    //aiming = 1;
    if(0 < targetDistance < 100){
      shooter((-shooterInput*(shooterAdjustment)));
      frc::SmartDashboard::PutString("targeting", "active");
      
      /*belt.Set(conveyorSpeed);
      lift1.Set(liftPower);
      lift2.Set(-liftPower);*/
    } else {
      shooter(-2000);
    }
    feed = 1;
  } else {
    shooter(0);
    frc::SmartDashboard::PutString("targeting", "inactive");
      //belt.Set(0);
      lift1.Set(0);
      lift2.Set(0);
      //aiming = 0;
    frc::SmartDashboard::PutString("SHOOTER:", "ACTIVE");
    feed = 0;
    shoot(0);
  } 
  /*
  if(logicontroller.GetRawButton(2)){
    //shoot(-0.5);
    belt.Set(-conveyorSpeed);
    lift1.Set(-liftPower);
    lift2.Set(liftPower);
  } else if(logicontroller.GetRawButton(1)){
    belt.Set(0.25);
    lift1.Set(0.25);
    lift2.Set(-0.25);
    frc::SmartDashboard::PutString("SHOOTER:", "ACTIVE");
  } else {

    belt.Set(0);
    lift1.Set(0);
    lift2.Set(0);

    frc::SmartDashboard::PutString("SHOOTER:", "INACTIVE");
  }
  */
  /*
  //  Conveyor/lift
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
  */
  //NON TEMPORARY:
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
  

  //Automatic Conveyor Belt Control
  
  if(logicontroller.GetRawButton(2)){
    belt.Set(-conveyorSpeed*2);
    lift1.Set(-liftPower);
    lift2.Set(liftPower);
    intake(-intakePower);
    frc::SmartDashboard::PutString("Spitting", "true");

  } else {
  //Running Intake Motors
    if(l_stick.GetRawButton(1)){
      intake(intakePower);
    } else if (logicontroller.GetRawButton(7)){
      intake(-intakePower);
    } else {
      intake(0);
    }
    frc::SmartDashboard::PutBoolean("Exit Sensor", sensorExit.Get());
    frc::SmartDashboard::PutBoolean("Intake Sensor", sensorIntake.Get());
    if((sensorIntake.Get() == 0 && sensorExit.Get() != 0) && aiming == 0){
        frc::SmartDashboard::PutString("CONVEYOR BELT:", "ACTIVE");
        belt.Set(conveyorSpeed);
    }else if(sensorIntake.Get() == 1 && aiming == 0) {
        belt.Set(0);
        frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
    } else {
      //belt.Set(0);
    }
    /*
    if(atSpeed){
      lift1.Set(liftPower); 
      lift2.Set(-liftPower);
    } else {
      lift1.Set(0);
      lift2.Set(0);
    }
    */
    frc::SmartDashboard::PutString("Spitting", "false");
  }
  frc::SmartDashboard::PutBoolean("shooting", feed);

  // MOVEMENT
  #ifdef arcadeDrive
    //Arcade Drive
    if (r_stick.GetRawButton(1)){
      leftDrive(-l_stick.GetY() - (0.5 * (r_stick.GetX())));
      rightDrive(-l_stick.GetY() + (0.5 *(r_stick.GetX())));
    }  else {
      leftDrive(l_stick.GetY() - (0.5 * (r_stick.GetX())));
      rightDrive(l_stick.GetY() + (0.5 *(r_stick.GetX())));
    }
  #else
    //Tank Drive
    leftDrive(l_stick.GetY());
    rightDrive(r_stick.GetY());
  #endif

  //Declaring Variables to store Table Information
  auto boxes = table->GetNumberArray("boxes", {});
  auto object_classes = table->GetStringArray("object_classes", {});

  //Shooter Control
  //  Getting Distance
  #ifdef portML
  //    USING MACHINE LEARNING
  for (int i; i < boxes.size; i+=4){
    if(object_classes[i] == "outer_port"){
      targetDistance = ((focalLength*outerPortH*limelightY)/((boxes[i+1]-boxes[i+3])*limelightHeight))+innerPortDepth;
    }
  }
  #else
  //    Using Limelight
  double llPortH = lltable->GetNumber("tvert", 0)*2;
  //frc::SmartDashboard::PutNumber("target height in pixels", llPortH);
  //targetDistance = 304.8*((focalLength*outerPortH*limelightY)/((llPortH)*limelightHeight))+innerPortDepth;
  double targetArea = lltable->GetNumber("ta", 0);
  frc::SmartDashboard::PutNumber("target area", targetArea);
  targetDistance = ((18)*(pow(targetArea, -.509)));
  horizontalOffset = lltable->GetNumber("tx", 0);
  frc::SmartDashboard::PutNumber("distance", targetDistance);

  #endif
  //  Getting RPM
  hoodAngle = atan((2*(73/12))/targetDistance);
  frc::SmartDashboard::PutNumber("angle", hoodAngle);
  frc::SmartDashboard::PutNumber("angle degrees", ((hoodAngle*180/M_PI)));
  frc::SmartDashboard::PutNumber("hood setpoint", hoodSetpoint);
  vFeetPerSecond = (2*(sqrt(((73/12)*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
  frc::SmartDashboard::PutNumber("Speed(ft/s", vFeetPerSecond);
  shooterRPM = 2*(vFeetPerSecond*(60*12))/(4*M_PI);
  //messed up
  shooterInput = shooterRPM/(1.25);
  frc::SmartDashboard::PutNumber("shooter RPM", shooterRPM);
  //  Convert Angle to Encoder Counts
  //    38 Counts ~ 15 Degrees
  hoodSetpoint = hoodAngle * hoodMax / angleRange;
  //  Update Encoders
  hoodPosition = hoodPoint.GetPosition();
  hoodPosition = hoodPosition - hoodInit;
  elevatorPosition = elevatorPoint.GetPosition();
  elevatorPosition = elevatorPosition - elevatorInit;
  turretPosition = turretPoint.GetPosition();
  turretPosition = turretPosition - turretInit;
  //Shooter Launch
  frc::SmartDashboard::PutNumber("aiming", aiming);

  //Intake Solenoid
  if(logicontroller.GetRawButtonPressed(6) && intakeSolUp == 0){
    intakeSolUp = 1;
    intakeSolOpen.Set(false);
    intakeSolClose.Set(true);
  }
  else if(logicontroller.GetRawButtonPressed(8) && intakeSolUp == 1){
    intakeSolUp = 0;
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
/*
elevator.Set(PID(elevatorSetPoint - , elevatorKp, elevatorKi));
  if(r_stick.GetRawButton(10)){
    //Extend Elevator
    elevatorSetPoint = elevatorUp;
  }
  else if (r_stick.GetRawButton(11)){
    //Climb
    elevatorSetPoint = elevatorIntermediate;
  }
  else if(r_stick.GetRawButton(9)){
    //Retract Elevator
    elevatorSetPoint = elevatorDown;
  }
  */
  
  

  //SOFT STOPS
  //  Soft stop for Elevator
  //  NEEDS elevatorUp AND elevatorDown VALUE
  /*
  if (elevatorPosition > elevatorUp || elevatorPosition < elevatorDown){
    elevator.Set(0);
  }
  */
  //  Soft stop for Hood
  if(hoodPosition >= -0.5){
    hood.Set(-0.2);
  } else if (hoodPosition <= hoodMax){
    hood.Set(0.2);
  } else {
    if(aiming == 0){
      hood.Set(0.2 * logicontroller.GetRawAxis(1));
    } else {
      hood.Set(PID(hoodSetpoint-hoodPosition, hoodKp, hoodKi));
    }
  }
  //temp test
  //hood.Set(-logicontroller.GetRawAxis(1));
  //  Soft stop for Turret
  if(turretPosition >= turretMax){
    turret.Set(-0.05);
  } else if (turretPosition <= -turretMax){
    turret.Set(0.05);
  } else {
    if(aiming == 1){
    //shooter(shooterRPM);
    //shooter(1);

      lltable->PutNumber("ledMode", 3);
    //Hood Up
    hood.Set(PID(hoodSetpoint-hoodPosition, hoodKp, hoodKi));
    //Turret Control
    turret.Set(-(PID(horizontalOffset, turretKp, turretKi)));
    } else {
    //shooter(0);
      lltable->PutNumber("ledMode", 1);
      turret.Set(0.25*-logicontroller.GetRawAxis(0));
    //Hood Down
    //hood.Set(PID(hoodDown-hoodPosition, hoodKp, hoodKi));
    }
  }

  //Skywalker Control
  skywalker.Set(ControlMode::PercentOutput, logicontroller.GetRawAxis(2)); 
  

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
    frc::SmartDashboard::PutNumber("Hood Position", hoodPosition);
    frc::SmartDashboard::PutNumber("Elevator Position", elevatorPosition);
    frc::SmartDashboard::PutNumber("Turret Position", turretPosition);
  #endif
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif