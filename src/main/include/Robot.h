/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:

 //CONFIGURATION
  //  Comment out for Tank Drive
    #define arcadeDrive
  //  Comment to disable Encoder output in SmartDashboard
    #define TestSetpoints
  //  Comment out to Disable Auto Ball Pickup
    //#define autoBallPickup
  //  Uncomment to control Auto aiming with Machine Learning rather than the Limelight.
    //#define portML

  //OUR FUNCTIONS
  /*
  double cotan(double i);
  double PID(double error, double Kp, double Ki);
  bool softStop(float max, float min, double motorInput, double motorPosition);
  void leftDrive(double power);
  void rightDrive(double power);
  float driveCurve(float input);
  void drive(float left, float right, bool intaking, bool reverse);
  void intake(double power);
  void syncShooters(double input);
  void turretSet(double error);
  void hoodSet(double error);
  void syncLift(double input);
  void shooterSubsystem(int mode);
  void teleop(int shooterMode, bool shooting, bool aiming, bool override); */

  //FRC Functions
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  //Button Names
  /*
  #define X 1
  #define A 2
  #define B 3
  #define Y 4
  #define lb 5
  #define rb 6
  #define lt 7
  #define rt 8
  #define back 9
  #define select 10
  #define l3 11
  #define r3 12
*/
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};