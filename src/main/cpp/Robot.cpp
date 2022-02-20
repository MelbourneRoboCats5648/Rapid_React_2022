// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>


// function for Emergancy Stop
void Robot::EmergencyStop() {
  m_motor_left.Set(0);
  m_motor_right.Set(0);
  motor_climb_up.Set(0);
  motor_climb_down.Set(0);
  motor_ball_intake.Set(0);
  motor_shooting_low.Set(0);
  motor_shooting_high.Set(0);
  }
  

// Settig up the robt
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

//_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

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
  m_encoder_right.SetDistancePerPulse(distance_per_pulse);
  m_encoder_left.SetDistancePerPulse(distance_per_pulse);
  m_encoder_right.Reset();
  m_encoder_left.Reset();
}

//_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  // check items above what there use is

  // turn on the motors (start the code)
  GoStraight(autonomous_backward_speed);
  double average_encoder_distance = (m_encoder_left.GetDistance() + m_encoder_right.GetDistance())/2.0;

  // if the robot has travelled over the required distance stop robot
  if (average_encoder_distance >= autonomous_distance_backward){
    GoStraight(0); // stopping the robot from moving

    // shooting
    Ball_Shooting(shooting_low_speed, shooting_high_speed);
    
    } 
    
}

//_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

void Robot::TeleopInit() {}

//_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

void Robot::TeleopPeriodic() { // repeated throughout 
  
  // if not stop run everything else
  if (m_controller.GetStartButtonPressed() || m_controller.GetBackButtonPressed()){ 
    // if the two small buttons in the middle of the x box controller are pressed stop everything
    EmergencyStop();
  }
  else{ // arcade drive & mechs
    Drive();
    Climb (climb_motor_speed);
    Ball_Intake(ball_intake_speed);
    Ball_Shooting(shooting_low_speed, shooting_high_speed);
  }


}


//_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _

void Robot::Drive()
{
  // Controls forward & backward movement using the left hand joystick
  // limited to a max speed by max_forward_speed setting
  double forward_input = m_controller.GetLeftY()* max_forward_speed;

  // Controls spin movement using the right hand joystick
  // limited to a max speed by max_spin_speed
  double spin_input = m_controller.GetRightX() * max_spin_speed;

  // if we are not trying to turn, we can start using encoders to make the robot go straight
  if (forward_input == 0)
  {
    // if m_going_forward is false, we were previously turning and need to zero encoders: continue driving as usual but do not use encoders
    // if m_going_forward is true, we have been going straight, use encoder assistance to GoStraight
    if (m_going_forward == false)
    {
      // reset encoders to zero
      m_encoder_right.Reset();
      m_encoder_left.Reset();
      // drive
      m_drive.ArcadeDrive(spin_input,0);
    }
    else {
      GoStraight(spin_input);
    }
    // we are trying to go straight, set m_going_forward to true
    m_going_forward = true;
  }
  else
  {
     m_drive.ArcadeDrive(spin_input, forward_input);
     m_going_forward = false;
  }
}

//________________________________________________________________________________________________________________________________

// Drive straight using ENCODERS
void Robot::GoStraight(double forwardSpeed)
{
  // defining variables for different distances
  double left_distance = m_encoder_left.GetDistance();
  double right_distance =  m_encoder_right.GetDistance();

  error = right_distance-left_distance; // error = difference btwn 2 distances

  if (error == 0){
    steering = 0; // this is straight continue on current trajectory 
  }
  else {
    steering = error * correction_factor; // setting steering value to a portion of the error
  }
  
  if (steering > max_turn_speed){
    steering = max_turn_speed;
  }
  else if (steering < -max_turn_speed){
    steering = -max_turn_speed;
  }

  
  m_drive.ArcadeDrive(forwardSpeed, steering);

}

//________________________________________________________________________________________________________________________________
void Robot:: Climb(double climb_motor_speed){

  if (m_controller.GetPOV() == 0){  // if top of the cross button is pressed
  // ---> climb expands/goes up
    motor_climb_up.Set(climb_motor_speed/3.2);
    motor_climb_down.Set(climb_motor_speed);
  }

  else if(m_controller.GetPOV() == 180){    // if bottom of the cross button is pressed
    // ---> climb expands/goes down
    motor_climb_up.Set(-climb_motor_speed/3.2);
    motor_climb_down.Set(-climb_motor_speed);
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
