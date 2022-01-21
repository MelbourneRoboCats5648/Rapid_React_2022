// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/drive_base_run.h"

drive_base_run::drive_base_run() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void drive_base_run::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void drive_base_run::Execute() {}

// Called once the command ends or is interrupted.
void drive_base_run::End(bool interrupted) {}

// Returns true when the command should end.
bool drive_base_run::IsFinished() {
  return false;
}
