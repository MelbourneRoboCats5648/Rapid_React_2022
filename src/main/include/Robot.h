// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// importing 
#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Encoder.h>
#include <frc/GenericHID.h>
#include <frc/DigitalOutput.h>
// importing,motors for mechanisms 
#include <frc/motorcontrol/PWMTalonSRX.h>



  
class Robot : public frc::TimedRobot {
 public:
  // defining methods/functions
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 //defining variable internal to robot --> (private)
 private:
  void EmergencyStop();
  void Drive();
  void GoStraight(double forwardSpeed);
  void Climb(double climb_motor_speed);
  void Ball_Intake(double ball_intake_speed);
  void Ball_Shooting(double shooting_low_speed, double shooting_high_speed);
  
  // probably sending info to game controller centre
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  frc::XboxController m_controller{0}; //defining xbox controller (port 0)

  // setting up vector motor controllers (driving)
  frc::PWMSparkMax m_motor_left{1}; 
  frc::PWMSparkMax m_motor_right{2};
  frc::Encoder m_encoder_left{0,1};
  frc::Encoder m_encoder_right{2,3};
  frc::PWMTalonSRX motor_climb_up {0};
  frc::PWMTalonSRX motor_climb_down {0};
  frc::PWMTalonSRX motor_ball_intake {0};
  frc::PWMTalonSRX motor_shooting_low {0};
  frc::PWMTalonSRX motor_shooting_high {0};
  frc::DigitalOutput solenoid {9};

  /*
  m_motor_left
  m_motor_right
  m_encoder_left
  m_encoder_right
  motor_climb_up 
  motor_climb_down 
  motor_ball_intake 
  motor_shooting_low
  motor_shooting_high

  */


  bool m_going_forward;
  
  // drive base defining 
  frc::DifferentialDrive m_drive{m_motor_left,m_motor_right};
  
  // defining aracde drive
  double max_forward_speed = 0.8; // maximum speed for forward & backward movement in drive
  double max_spin_speed = 0.8; // maximum speed for spin movement

  // defining speeds for mechs
  double ball_intake_speed = 0.5; //ball intake speed (button A)
  double shooting_low_speed = 0.5; //ball intake speed (button B) shared & synced with shooting high
  double shooting_high_speed = 0.5; //ball intake speed (button B)shared & synced with shooting low
  double climb_motor_speed = 0.5; // climb motor speed (only one bc should be the same speed)

  double distance_per_pulse = 11.9694680102; // circumfrance = 2πr,  r = 3inch --> in relation to drive wheels. circumfrance/ 4 = distance by pulse
  double autonomous_backward_speed = -0.15; //make sure that this is NEGATIVE 

  // autonomous distance
  double autonomous_distance_backward = 100; //cm

  // driving straight w/ encoders variables
  double error = 0;
  double steering = 0;
  double correction_factor = 0.15; // this refers to a proportion of the error
  double max_turn_speed = 0.6;


};
