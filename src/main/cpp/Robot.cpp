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

#define TestSetpoints
double hoodPosition;
double elevPosition;
double turretPosition;

bool colorWheelUp = 1;
bool elevBreakUp = 1;
bool solUp = 1;

//CONSTANTS & OTHER GLOBAL VARIABLES
//FUNCTIONAL
//Power of Intake
#define intakePower 0.5
#define conveyorSpeed 0.5
#define liftPower 0.5
#define colorWheelPower 0.5
#define RPM 2000

//For Testing PID Setpoints
string pointTest = "";

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

//SparkMax Motor Declaration
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

//Solenoids
frc::Compressor compressor { 0 };
//One of these is up and the other is down ?
frc::Solenoid intakeSolOpen { 0 };
frc::Solenoid intakeSolClose { 1 };
frc::Solenoid colorWheelSolOpen { 2 };
frc::Solenoid colorWheelSolClose { 3 };
frc::Solenoid elevBreakOpen { 4 };
frc::Solenoid elevBreakClose { 5 };


//CONTROLLERS

frc::Joystick r_stick  {0};
frc::Joystick l_stick  {1};
frc::Joystick logicontroller {2};

//FUNCTIONS

//Sync left wheel motors
void leftDrive(double power){
  leftFrontFalcon.Set(ControlMode::PercentOutput, power);
  leftBackFalcon.Set(ControlMode::PercentOutput, power);
  cout << "LEFT WHEELS IN MOTION";
}
//Sync right wheel motors
void rightDrive(double power){
  rightFrontFalcon.Set(ControlMode::PercentOutput, -power);
  rightBackFalcon.Set(ControlMode::PercentOutput, -power);
  cout << "RIGHT WHEELS IN MOTION";
}

void shooter(int power){
  //l_shooter.Set(ControlMode::Velocity, power * RPM * 4096 / 600);
  //r_shooter.Set(ControlMode::Velocity, power * RPM * 4096 / 600);
  //cout << "SHOOTER RUNNING AT" << RPM << "RPM";
  l_shooter.Set(ControlMode::PercentOutput, power*0.2);
  r_shooter.Set(ControlMode::PercentOutput, power*-0.2);
}

void lift(double power){
  lift1.Set(power);
  lift2.Set(-power);
}

//Upon robot startup
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Set motors off initially
  leftFrontFalcon.Set(ControlMode::PercentOutput, 0);
  leftBackFalcon.Set(ControlMode::PercentOutput, 0);
  rightFrontFalcon.Set(ControlMode::PercentOutput, 0);
  rightBackFalcon.Set(ControlMode::PercentOutput, 0);
  l_shooter.Set(ControlMode::PercentOutput, 0);
  r_shooter.Set(ControlMode::PercentOutput, 0);
  colorWheelMotor.Set(0);
  turret.Set(0);
  belt.Set(0);
  frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  lift1.Set(0);
  lift2.Set(0);
  MCGintake.Set(ControlMode::PercentOutput, 0);
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

  //Movement
  #ifdef arcadeDrive
    //Arcade Drive
    leftDrive(l_stick.GetY() - (0.5 * (r_stick.GetX())));
    rightDrive(l_stick.GetY() + (0.5 *(r_stick.GetX())));
  #else
    //Tank Drive
    leftDrive(l_stick.GetY());
    rightDrive(r_stick.GetY());
  #endif

  //Conveyor belt
  if(logicontroller.GetRawButton(6)){ 
    belt.Set(conveyorSpeed);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "ACTIVE");
    cout << "CONVEYOR ACTIVE";
  }
  else if(logicontroller.GetRawButton(8)){
    belt.Set(-conveyorSpeed);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "REVERSE");
    cout << "CONVEYOR REVERSED";
  }
  else {
    belt.Set(0);
    frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  }

  //DELCARE INTAKE AS - POWER, EXTAKE AS + POWER
  //Intake
  if (logicontroller.GetRawButton(5)){
    MCGintake.Set(ControlMode::PercentOutput, conveyorSpeed);
    cout << "MCGINTAKE ACTIVE";
  }
  else if (logicontroller.GetRawButton(7)){
    MCGintake.Set(ControlMode::PercentOutput, -conveyorSpeed);
    cout << "MCGINTAKE REVERSED";
  }
  else {
    MCGintake.Set(ControlMode::PercentOutput, 0);
  }

  //Intake Solenoid
  if(logicontroller.GetRawButton(2) && solUp == 0){
    solUp = 1;
    intakeSolOpen.Set(false);
    intakeSolClose.Set(true);
    cout << "SOLENOID UP";
  }
  else if(logicontroller.GetRawButton(2) && solUp == 1){
    solUp = 0;
    intakeSolOpen.Set(true);
    intakeSolClose.Set(false);
    cout << "SOLENOID DOWN";
  }

  //Color Wheel Solenoid
  if(r_stick.GetRawButton(3) && colorWheelUp == 0){
    colorWheelUp = 1;
    colorWheelSolOpen.Set(true);
    colorWheelSolClose.Set(false);
    cout << "COLOR WHEEL UP";
  }
  else if(r_stick.GetRawButton(3) && colorWheelUp == 1){
    colorWheelUp = 0;
    colorWheelSolOpen.Set(false);
    colorWheelSolClose.Set(true);
  }

  //Shooter
  if(logicontroller.GetRawButton(1)){
    shooter(1);
  }
  else {
    shooter(0);
  }

  //Turret
  turret.Set(logicontroller.GetRawAxis(2));

  //Lift
  if(logicontroller.GetRawButton(3)){
    lift(liftPower);
  }
  else if(logicontroller.GetRawButton(10)){
    lift(-liftPower);
  }
  else {
    lift(0);
  }

  //Color Wheel Motor
  if(logicontroller.GetRawButton(4)){
    colorWheelMotor.Set(colorWheelPower);
  }

  

  //Skywalker
  if(l_stick.GetRawButton(4)){
    skywalker.Set(ControlMode::PercentOutput, 0.2);
  }
  else if (l_stick.GetRawButton(5)){
    skywalker.Set(ControlMode::PercentOutput, -0.2);
  }

  //Hood
  hood.Set(-logicontroller.GetRawAxis(1)*0.25);

  //Elevator
  elevator.Set(logicontroller.GetRawAxis(0)*0.3);

  /*  THE CODE BELOW IS FOR FINDING SETPOINTS FOR PID
      COMMENT IT OUT UNLESS WE NEED TO REDO PID SETPOINTS */
  #ifdef TestSetpoints
  hoodPosition = hoodPoint.GetPosition();
  elevPosition = elevPoint.GetPosition();
  turretPosition = turretPoint.GetPosition();
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