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
  motor_climb_up.Set(0);
  motor_climb_down.Set(0);
  motor_ball_intake.Set(0);
  motor_shooting_low.Set(0);
  motor_shooting_high.Set(0);
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
  // random stuff review later
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  //________________________________________________________________________________

  solenoid.Set(true);
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
  Drive();
  Climb (climb_motor_speed);
  Ball_Intake(ball_intake_speed);
  Ball_Shooting(shooting_low_speed, shooting_high_speed);


  // determine EStop button
  // if (m_controller.GetRawButtonPressed(10)) {EmergencyStop();}

}

// Gets left hand and right hand joystick input to control the drive base
// If not trying to turn, will use drive encoders to make the robot go straight
void Robot::Drive()
{
  // Controls forward & backward movement using the left hand joystick
  // limited to a max speed by max_forward_speed setting
  double forward_input = m_controller.GetLeftY()* max_forward_speed;

  // Controls spin movement using the right hand joystick
  // limited to a max speed by max_spin_speed
  double spin_input = m_controller.GetRightX() * max_spin_speed;

  // if we are not trying to turn, we can start using encoders to make the robot go straight
  if (spin_input == 0)
  {
    // if m_going_forward is false, we were previously turning and need to zero encoders: continue driving as usual but do not use encoders
    // if m_going_forward is true, we have been going straight, use encoder assistance to GoStraight
    if (m_going_forward == false)
    {
      // reset encoders to zero
      m_encoder_right.Reset();
      m_encoder_left.Reset();
      // drive
      m_drive.ArcadeDrive(forward_input, 0);
    }
    else {
      GoStraight(forward_input);
    }
    // we are trying to go straight, set m_going_forward to true
    m_going_forward = true;
  }
  else
  {
     m_drive.ArcadeDrive(forward_input,spin_input);
     m_going_forward = false;
  }
}

// Drive straight
void Robot::GoStraight(double forwardSpeed)
{
    // get encoder measurements since last check
    double rightEncoderDistance = m_encoder_right.GetDistance();
    double leftEncoderDistance = m_encoder_left.GetDistance();
    // transform the difference between the left and right distances into a spin correction
    // ASSUMING the difference is small between left and right (i.e., 3-4 ticks) 
    // "right / left" will be a value between [0, 2]
    // "right / left - 1" will be a value between [-1, 1]
    // if left distance is less than right distance, the spin correction will be positive (clockwise)
    // if right distance is less than left distance, the spin correction will be negative (anticlockwise)
    double spinCorrection = (rightEncoderDistance/leftEncoderDistance) - 1;
    m_drive.ArcadeDrive(forwardSpeed, spinCorrection);
    //reset encoders to zero
    m_encoder_right.Reset();
    m_encoder_left.Reset();
}

//________________________________________________________________________________________________________________________________
void Robot:: Climb(double climb_motor_speed){

  if (m_controller.GetPOV() == 0){  // if top of the cross button is pressed
  // ---> climb expands/goes up
    motor_climb_up.Set(climb_motor_speed);
    motor_climb_down.Set(-climb_motor_speed);
  }

  else if(m_controller.GetPOV() == 180){    // if bottom of the cross button is pressed
    // ---> climb expands/goes down
    motor_climb_up.Set(-climb_motor_speed);
    motor_climb_down.Set(climb_motor_speed);
    }
  
  else{
    motor_climb_up.Set(0);
    motor_climb_down.Set(0);
    }
  }
  


//________________________________________________________________________________________________________________________________
void Robot:: Ball_Intake(double ball_intake_speed){

  if (m_controller.GetAButton() == true){
    motor_ball_intake.Set(ball_intake_speed);}
  else { // stopping movement
    motor_ball_intake.Set(0);
    }
}
//________________________________________________________________________________________________________________________________

void Robot:: Ball_Shooting(double shooting_low_speed, double shooting_high_speed){

  if (m_controller.GetBButton() == true){
    motor_shooting_low.Set(shooting_low_speed);
    motor_shooting_high.Set(shooting_high_speed);}
  else {// stopping movement
    motor_shooting_low.Set(0);
    motor_shooting_high.Set(0);}
    
  }
//________________________________________________________________________________________________________________________________

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

// testing stuff
void Robot::TestInit() {}

void Robot::TestPeriodic() {
  
  }



//_____________________________________________________________
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
