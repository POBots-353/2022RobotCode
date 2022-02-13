// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetDistanceCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  double neededDistance;

  public SetDistanceCommand(DriveSubsystem drive, double neededDistance) {
    driveSubsystem = drive;
    this.neededDistance = neededDistance;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //Moves the robot based on the error(expected distance - current distance)
    if (Math.abs(driveSubsystem.distanceError(neededDistance)) > 3) {
      driveSubsystem.manualDrive(0.0, -driveSubsystem.distanceError(neededDistance) * 0.5, 0.0, 35);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    //If the distance is with in 3cm of desired point, then end command
    if (Math.abs(driveSubsystem.distanceError(neededDistance)) < 3) {
      return true;
    }
    return false;
  }
}
