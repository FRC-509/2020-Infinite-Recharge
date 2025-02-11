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
#include <frc/WPILib.h>
#include <cameraserver/CameraServer.h>
#include <vision/VisionRunner.h>

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
#define conveyorSpeed 0.5
//Lift Properties
#define liftPower 0.5
//Hood Properties
#define angleRange M_PI/12
//Rpm of Shooter
double shooterRPM = std::clamp(0, 0, 80000);
double targetDistance;
double vFeetPerSecond;
#define shooterAdjustment 1.05
//Elevator
double elevatorPosition;
//Turret Soft Stop + Hood Soft Stop
double turretInit;
double hoodInit;
double elevatorInit;
#define turretMax 105
#define hoodMax -40
#define hoodMin 0

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
#define turretKp 0.05
#define turretKi 0
#define minOffset 10
#define turretDefaultSetpoint 0
double turretPosition;
double horizontalOffset;
double shooterInput;

//Input Tracking
double turretInput;
double hoodInput;

//  State Tracking Variables
//Starting Mode of Solenoid
bool intakeSolUp = 1;
bool bsolUp = 1;
//Powercell Pickup mode (0 for manual, 1 for auto)
bool pickupMode = 0;
//Belt Modes
int shooterMode = 0;

int LEDPWM;

bool shootCommand;
bool manualToggle;
//colors
string blue = "Blue";
string red = "Red";
string yellow = "Yellow";
string green = "green";
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
rev::CANEncoder turretEncoder = turret.GetEncoder();
//Hood controls angle of Shooter
rev::CANSparkMax hood { 13 , rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder hoodEncoder = hood.GetEncoder();
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

//Drive Functions
//  Sync left and right wheel motors
void leftDrive(double power){
  leftFrontFalcon.Set(ControlMode::PercentOutput, power);
  leftBackFalcon.Set(ControlMode::PercentOutput, power);
}
void rightDrive(double power){
  rightFrontFalcon.Set(ControlMode::PercentOutput, -power);
  rightBackFalcon.Set(ControlMode::PercentOutput, -power);
}
//  Drive smoothing
float driveCurve(float input){
  float output;
  float linearity = 0.5;
  output = linearity * input + (1-linearity) * pow(input, 3);
  return output;
}
//Intake motors
void intake(double power, bool intakeSol){
  //Setting intake power
  MCGintake.Set(ControlMode::PercentOutput, -power);
  //Setting intake position
  if (intakeSol){
    intakeSolOpen.Set(false);
    intakeSolClose.Set(true);
  } else {
    intakeSolOpen.Set(true);
    intakeSolClose.Set(false);
  }
}
//  Combined Drive Function
void drive(float left, float right, bool intaking, bool reverse, bool climb){
  //driving
  if (!climb){
    if (!reverse){
      leftDrive(driveCurve(left) - driveCurve(right));
      rightDrive(driveCurve(left) + driveCurve(right));
    } else {
      leftDrive(-driveCurve(left) - driveCurve(right));
      rightDrive(-driveCurve(left) + driveCurve(right));
    }

    //intaking
    if (intaking){
      intakeSolUp = 1;
      intake(intakePower, intakeSolUp);
    } else {
      intakeSolUp = 0;
      intake(0, intakeSolUp);
    }
  }
  //Output Intake Position to SmartDashboard
  frc::SmartDashboard::PutBoolean("Intake Down?", intakeSolUp);
}

//Sync Shooter Motors
bool syncShooters(double input){
  l_shooter.Set(ControlMode::Velocity, input * (shooterAdjustment) * 4096/600);
  r_shooter.Set(ControlMode::Velocity, -1 * input * (shooterAdjustment) * 4096/600);
  if (input > 0 || input < 0){
    return true;  
  } else {
    return false;
  }
}
//changing the color wheel spinners position
void PosChangeColorWheel(){
  if(r_stick.GetRawButton(2)){
    colorSolUp.Set(false);
    colorSolDown.Set(true);
  } else {
    colorSolUp.Set(true);
    colorSolDown.Set(false);
  }
}
//Sync and reverse a kicker motor
void syncLift(double input){
    lift1.Set(input);
    lift2.Set(-input);
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
//Soft Stop Function (true should stop)
bool softStop(float max, float min, double motorInput, double motorPosition){
  if ((-motorInput >= max && motorPosition > 0) || (-motorInput <= min && motorPosition < 0)){
    return true;
  } else {
    return false;
  }
}

/*char colorSensor(){

  //Color Sensor calculations
  frc::Color detectedColor = m_colorSensor.GetColor();
  string colorString;
     double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
  //Set colorString to match detected color
  if (matchedColor == kBlueTarget) {
    colorString = "Blue";
    char color = colorString[0];
    return color; 
  }
  else if (matchedColor == kGreenTarget) {
    colorString = "Green";
    char color = colorString[0];
    return color;
  }
  else if (matchedColor == kRedTarget) {
    colorString = "Red";
    char color = colorString[0];
    return color;
  }
  else if (matchedColor == kYellowTarget) {
    colorString = "Yellow";
    char color = colorString[0];
    return color;
  }
  else {
    colorString = "Unknown";
    char color = colorString[0];
    return color;
  }/*
  //Display color data on SmartDashboard
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Color", colorString);


}
/*void ColorWheel(char actualColor){
  char actualBlue = blue[0];
  char actualRed = red[0];
  char actualGreen = green[0];
  char actualYellow = yellow[0];
  if (actualColor == actualBlue){
      colorWheelMotor.Set(1);
  }
  else{
    colorWheelMotor.Set(0);
  }
  if (actualColor == actualGreen){
      colorWheelMotor.Set(1);
  }
  else{
    colorWheelMotor.Set(0);
  }
  if (actualColor == actualYellow){
      colorWheelMotor.Set(1);
  }
  else{
    colorWheelMotor.Set(0);
  }
  if (actualColor == actualRed){
      colorWheelMotor.Set(1);
  }
  else{
    colorWheelMotor.Set(0);
  }
}
*/
//Turret Movement
bool turretSet(double input){
  
  bool atSoftStop;
    
  if (softStop(-turretMax, turretMax, input, turretPosition)){
    turret.Set(0);
    atSoftStop = 1;
    return atSoftStop;
  } else {
    turret.Set(PID(input-turretPosition, hoodKp, hoodKi));
    atSoftStop = 0;
    return atSoftStop;
  }

  frc::SmartDashboard::PutBoolean("Turret at soft stop?", atSoftStop);
}
//Hood Movement
bool hoodSet(double input){
  /*
  bool atSoftStop;
    
  if (softStop(hoodMax, hoodMin, input, hoodPosition)){
    hood.Set(0);
    atSoftStop = 1;
    return atSoftStop;
  } else {
    hood.Set(PID(hoodSetpoint-hoodPosition, hoodKp, hoodKi));
    atSoftStop = 0;
    return atSoftStop;
  }*/

  //hood.Set(input);
  
  //frc::SmartDashboard::PutBoolean("Hood at soft stop?", atSoftStop);
}

//Turn Limelight LED on or off
bool limelightOn(bool ledState){
    if(ledState){
      lltable->PutNumber("ledMode", 3);
      return true;
    } else {
      lltable->PutNumber("ledMode", 1);
      return false;
    }
}

bool limelightTargetAquired(){
    bool targetAquired;
    targetAquired = lltable->GetNumber("tv", 0);
    if (targetAquired == 1){
      return true;
    } else {
      return false;
    }
}

//Limelight Output Function
float limelightOutput(int desiredOutput){
    
    bool errorState;
    
    float targetHorizontalDisplacement;
    float targetHeight;
    float targetArea; //in percent(1% = 1)    
    float targetVisible;
    switch(desiredOutput){
      //outputting target x value
      case 0:
          targetHorizontalDisplacement = lltable->GetNumber("tx", 0);
          return targetHorizontalDisplacement;
          errorState = 0;
          break;
      //outputting target y value
      case 1:
          targetHeight = lltable->GetNumber("ty", 0);
          return targetHeight;
          errorState = 0;
          break;
      //outputting target area value(only used for inaccurate distance aquisition)
      case 2:
          targetArea = lltable->GetNumber("ta", 0);
          return targetArea;
          errorState = 0;
          break;
      case 3:
          targetVisible = lltable->GetNumber("tv", 0);
          return targetVisible;
          errorState = 0;
          break;
      default:
          errorState = 1;
          break;

    }
    frc::SmartDashboard::PutNumber("tx", targetHorizontalDisplacement);
    frc::SmartDashboard::PutNumber("ty", targetHeight);
    frc::SmartDashboard::PutBoolean("Limelight Output Error?", errorState);
}

double distanceCalculator(){
  
  double targetArea;
  targetArea = limelightOutput(2);
  //distance by target area(needs work)
  targetDistance = ((18)*(pow(targetArea, -.509)));
  frc::SmartDashboard::PutNumber("target distance calculated:", targetDistance);
  return targetDistance;
 
}        
        
void conveyor(int mode, bool shooting){
  bool conveyor;
  bool manual;
  bool errorState;
  switch(mode){
    //auto control(normal)
    case 0:
      if(!sensorIntake.Get() && sensorExit.Get()){
        belt.Set(conveyorSpeed);
        conveyor = 1;
      } else {
        belt.Set(0);
        conveyor = 0;
      }
      syncLift(0);
      manual = 0;
      errorState = 0;
      break;      
    //shoot(going towards shooter)
    case 1:
      belt.Set(conveyorSpeed);
      syncLift(liftPower);
      manual = 0;
      errorState = 0;
      break;
    //poot(going towards intake)
    case 2:
      belt.Set(-conveyorSpeed);
      syncLift(-liftPower);
      manual = 0;
      errorState = 0;
      break;
    //manual control (uncomplete)
    case 3:
      if(!shooting){
        belt.Set(-logicontroller.GetRawAxis(3));
        syncLift(-logicontroller.GetRawAxis(3));
      } else {
        belt.Set(-logicontroller.GetRawAxis(3));
        syncLift(-logicontroller.GetRawAxis(3));
      }
      manual = 1;
      errorState = 0;
      break;
    default:
      errorState = 1;
      break;
  }
    
    //output
    frc::SmartDashboard::PutBoolean("Conveyor Belt On?:", conveyor);
    frc::SmartDashboard::PutBoolean("Manual Control On?:", manual);
    frc::SmartDashboard::PutBoolean("Full?", !sensorExit.Get());
}
//turretTracking function, outputs true if at position
bool turretTracking(){
  double horizontalOffset;
  horizontalOffset = limelightOutput(0);
  turret.Set(PID(-horizontalOffset, turretKp, turretKi));
  //turretSet((0 - horizontalOffset));
  if (5 <= horizontalOffset && horizontalOffset <= 5){
    return true;
  } else {
    return false;
  }
  frc::SmartDashboard::PutNumber("horizontal offset", horizontalOffset);
}
//hoodTracking function, outputs true if at position    
bool hoodTracking(){
  double hoodTarget;
  double hoodTargetDegrees;
  double hoodTargetEncoder;
  double error;
  hoodTarget = atan((2*(73/12))/distanceCalculator());
  hoodTargetDegrees = ((hoodTarget/M_PI)*180);
  hoodTargetEncoder = ((-2 * hoodTargetDegrees) + 100);
  //50deg to 70deg(hood) 
  //hood.Set(logicontroller.GetRawAxis(1));
  
  //hood.Set(PID((hoodTargetEncoder - hoodEncoder.GetPosition()), hoodKp, hoodKi));
  if (-0.1 <= error && error <= 0.1){
    return true;  
  } else {
    return false;
  }  
  frc::SmartDashboard::PutNumber("hoodTarget", hoodTarget);
}
//aiming function(aims, and outputs if it's aimed)
bool aiming(bool manual){
  
  bool aimed;
  if(!manual){  
    if ((turretTracking()) && hoodTracking()){
      aimed = 1;
      return aimed;
    } else {
      aimed = 0;
      return aimed;
    }
  } else {
    turretSet(-logicontroller.GetRawAxis(0));
    hoodSet(-logicontroller.GetRawAxis(1));
  }
  frc::SmartDashboard::PutBoolean("aimed?", aimed);
}
double shooterSpeed(){
  double hoodAngle;
  hoodAngle = atan((2*(73/12))/distanceCalculator());
  //hoodAngle = (50*M_PI)/180;
  //hoodAngle in radians
  vFeetPerSecond = (2*(sqrt(((73/12)*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
  shooterInput = shooterRPM/(1.25);
  return shooterInput;
  frc::SmartDashboard::PutNumber("feet per second", vFeetPerSecond);
}
//shooter functionality
double shooterSubsystem(int mode, bool shootCommand){
  if(mode == 3){
    if (shootCommand){
      limelightOn(1);
      mode = 1;
    } else {
      limelightOn(0);
      mode = 0;
    }
  } else {
    if (shootCommand){
      limelightOn(1);
      mode = 1;
    } else {
      limelightOn(0);
      mode = 0;
    }
  }
  
  //in development
  //bool shootCommand
  //shootCommand = logicontroller.GetRawButton(8);
  bool aimCommand;
  bool manualControl;
  bool conveyorOn;
  bool spitting;
  bool shooterOn;
  bool aimMode;
  if (mode == 3){
    aimMode = 1;
  } else {
    aimMode = 0;
  }
  bool targetFound;
  targetFound = limelightTargetAquired();  
    
  double shooterInput;
  shooterInput = shooterSpeed();
  
  //
  switch(shooterMode){
    
    //Intake
    case 0:
      conveyor(0, shootCommand);
      manualControl = 0;
      break;
    //Shooting
    case 1:
      conveyor(1, shootCommand);
      if(aiming(mode) && shootCommand){
        syncShooters(shooterInput);
      } else {
        syncShooters(0);
      }
      manualControl = 0;
      break;
    //Spit
    case 2:
      shooterOn = syncShooters(0);
      conveyor(2, shootCommand);
      manualControl = 0;
      break;
    //"Manual Control"
    case 3:
      turret.Set(logicontroller.GetRawAxis(0));
      hoodSet(logicontroller.GetRawAxis(1));
      conveyor(3, shootCommand);
      //shooterOn = syncShooters(3250);
      manualControl = 1;
      break;    
  }
  
  if ((aiming(aimMode) || manualControl) && shootCommand){
    if(manualControl){    
      shooterOn = syncShooters(2750);
      //temp fix
      conveyor(3, shootCommand);
      conveyorOn = true;
    } else {
      shooterOn = syncShooters(shooterInput);
      //temp fix
      conveyor(3, shootCommand);
      conveyorOn = true;
    }
  } else {
    syncShooters(0);
    conveyorOn = false;
  }
  //smart dashboard output
  frc::SmartDashboard::PutBoolean("SHOOTER ON?:", shooterOn);
  frc::SmartDashboard::PutBoolean("Manual Control?", manualControl);
  frc::SmartDashboard::PutBoolean("CONVEYOR BELT:", conveyorOn);
  frc::SmartDashboard::PutNumber("shooter mode for real", mode);
  return shooterInput;
}
void climber(double elevatorPosition, bool manual){
  
  double upError;
  double downError;
  upError = elevatorUp - elevatorPosition;
  downError = elevatorDown - elevatorPosition;
  //Manual Elevator control
  if(manual){
    if(l_stick.GetRawButton(3)){
      brakeSolOff.Set(true);
      brakeSolOn.Set(false);
      elevator.Set(l_stick.GetRawAxis(1));
    } else {
      elevator.Set(0);
      brakeSolOff.Set(false);
      brakeSolOn.Set(true);
    }
  } else {
    //experimental PID
    if(l_stick.GetRawButton(2)){
      if(1 > upError > -1){
        brakeSolOff.Set(true);
        brakeSolOn.Set(false);
        elevator.Set(PID(upError, elevatorKp, elevatorKi));
      } else {
        brakeSolOff.Set(false);
        brakeSolOn.Set(true);        
        elevator.Set(0);
      }
    } else if(l_stick.GetRawButton(8)) {
      if(1 > downError > -1){
        brakeSolOff.Set(true);
        brakeSolOn.Set(false);
        elevator.Set(PID(downError, elevatorKp, elevatorKi));
      } else {
        brakeSolOff.Set(false);
        brakeSolOn.Set(true);        
        elevator.Set(0);
      }
    } else {
      elevator.Set(0);
      brakeSolOff.Set(false);
      brakeSolOn.Set(true);
    }

  }
  //Skywalker Control
  skywalker.Set(ControlMode::PercentOutput, logicontroller.GetRawAxis(2)); 
  
}
//all teleop controls(in development)
/*void teleop(int shooterMode, bool shooting, bool aiming, bool override){
  if (!override){
    shooterSubsystem(shooterMode, shooting, aiming);
    climber();
    //colorWheel();
  } else {
    shooterSubsystem(3, shooting, aiming);
    //override();
  }
}
*/
//Upon robot startup
void Robot::RobotInit() {
  //CURRENT LIMITING
  //limelightOn(1);  
  //Current Limiting SparkMAX
  lift1.SetSmartCurrentLimit(10);
  lift2.SetSmartCurrentLimit(10);
  belt.SetSmartCurrentLimit(10);
  turret.SetSmartCurrentLimit(10);
  hood.SetSmartCurrentLimit(10);
  elevator.SetSmartCurrentLimit(60);
  colorWheelMotor.SetSmartCurrentLimit(10);
  
  //Current Limiting Falcons + Intake (WIP)
  cs::UsbCamera usbcamera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  cs::UsbCamera usbcamera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  //Adding Camera Feeds
  usbcamera.SetResolution(1,1);
  usbcamera.SetFPS(15);
  usbcamera2.SetResolution(1,1);
  usbcamera2.SetFPS(15);
  
    
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //home turret encoder(put on checklist)
  turretInit = turretEncoder.GetPosition();
  hoodInit = hoodEncoder.GetPosition();
  elevatorInit = elevatorPoint.GetPosition();
  //  Update Encoders
  turretPosition = turretEncoder.GetPosition() - turretInit;
  hoodPosition = hoodEncoder.GetPosition() - hoodInit;
  elevatorPosition = elevatorPoint.GetPosition() - elevatorInit;
  //Turn off Limelight LED
  limelightOn(0);

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
 

 // frc::SmartDashboard::PutNumber("Resolution: ", usbcamera.getResolution());

 //usbcamera2.SetResolution(1,1);
 //usbcamera2.SetFPS(25);

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
  //limelightOn(1);
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
  double shooterInput;
  shooterInput = shooterSubsystem(shooterMode, shootCommand);
  frc::SmartDashboard::PutNumber("shoot inpoot", shooterInput);
  hood.Set(logicontroller.GetRawAxis(1));
  PosChangeColorWheel();
  bool butReallyShootCommand;
  frc::SmartDashboard::PutNumber("target distance calculated:", distanceCalculator());
  frc::SmartDashboard::PutNumber("poot spood", shooterSpeed());
  climber(elevatorPosition, true);
  //shoot command
  shootCommand = logicontroller.GetRawButton(8);
  butReallyShootCommand = logicontroller.GetRawButton(4);
  //manual toggle
  if(logicontroller.GetRawButton(9) && manualToggle == 0){
    shooterMode = 3;
    manualToggle = 1;
  } else if (logicontroller.GetRawButton(10) && manualToggle == 1){
    if(butReallyShootCommand){
      shooterMode = 1;
    } else {
      shooterMode = 0;
    }
    manualToggle = 0;
  }
  frc::SmartDashboard::PutBoolean("manual?", manualToggle);
  //shooterMode = 3;
  frc::SmartDashboard::PutBoolean("actuallyshoot buuton", butReallyShootCommand);
  frc::SmartDashboard::PutNumber("shooter modse", shooterMode);
  //shooterMode = 3;

  /*
  if(logicontroller.GetRawButton(2)){
    shooterMode = 1;
  } else if (logicontroller.GetRawButton(4)){
    shooterMode = 2;
  } else if (logicontroller.GetRawButton(1)){
    shooterMode = 3;
  } else {
    shooterMode = 0;
  }
  */

  //NEW Drive Control
  //forward/backward input, turn input, reverse?, intake?
  drive(l_stick.GetY(), r_stick.GetX(), r_stick.GetRawButton(1), l_stick.GetRawButton(1), l_stick.GetRawButton(3));
  shooterSubsystem(shooterMode, shootCommand);
  //NEW Conveyor Control  
  //mode, shooting?, aiming?, manual override?
  //teleop(shooterMode, logicontroller.GetRawButton(2), logicontroller.GetRawButton(3), logicontroller.GetRawButton(2));
  //Color Wheel Extension and Retraction

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