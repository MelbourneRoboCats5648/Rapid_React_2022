// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>


void Robot::EmergencyStop() {
  // stops all motors from moving 
  m_motor_left.Set(0);
  m_motor_right.Set(0);
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() { // repeated throughout 
  // arcade drive motor control

  // controls forward & backward movement
  double forward_input = m_controller.GetLeftY() * max_forward_speed;
  //controls spin movement 
  double spin_input = m_controller.GetRightX() * max_spin_speed;
  // controlling arcade drive
  if (spin_input == 0)
  {
    // TODO: check flag here
    double error = (m_encoder_right.GetDistance() - m_encoder_left.GetDistance())/(m_encoder_left.GetDistance()+m_encoder_right.GetDistance());
    m_drive.ArcadeDrive(forward_input,error);
    m_encoder_right.Reset();
    m_encoder_left.Reset();
    m_going_forward = true;
  }
  else
   {m_drive.ArcadeDrive(forward_input,spin_input);
   m_going_forward = false; }


  // if xbox button is pressed, stop everything 
  // TODO check if button 10 is actually the xbox guide button
  if (m_controller.GetRawButtonPressed(10)) {EmergencyStop();}

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

// testing stuff
void Robot::TestInit() {}

void Robot::TestPeriodic() {
  if (m_controller.GetRawButtonPressed(9)) fmt::print("Button 9 is pressed\n");
  if (m_controller.GetRawButtonPressed(10)) fmt::print("Button 10 is pressed\n");
  if (m_controller.GetRawButtonPressed(11)) fmt::print("Button 11 is pressed\n");
  if (m_controller.GetRawButtonPressed(12)) fmt::print("Button 12 is pressed\n");
  if (m_controller.GetRawButtonPressed(13)) fmt::print("Button 13 is pressed\n");
  if (m_controller.GetRawButtonPressed(14)) fmt::print("Button 14 is pressed\n");
  if (m_controller.GetRawButtonPressed(15)) fmt::print("Button 15 is pressed\n");
  if (m_controller.GetRawButtonPressed(16)) fmt::print("Button 16 is pressed\n");
  }



//_____________________________________________________________
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
