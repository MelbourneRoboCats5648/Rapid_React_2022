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

  bool m_going_forward;
  
  // drive base defining 
  frc::DifferentialDrive m_drive{m_motor_left,m_motor_right};
  
  // defining aracde drive
  double max_forward_speed = 0.8; // maximum speed for forward & backward movement in drive
  double max_spin_speed = 0.8; // maximum speed for spin movement
};
