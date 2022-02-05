// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  double neededDistance;
  double neededAngle;
  /**This command uses the UltraSonic sensor and gryo to create a swerve */
  public SwerveCommand(DriveSubsystem drive, double neededAngle, double neededDistance) {
    driveSubsystem = drive;
    this.neededAngle = neededAngle;
    this.neededDistance = neededDistance;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(driveSubsystem.distanceError(neededDistance)) > 3 && Math.abs(driveSubsystem.angleError(neededAngle)) > 1) {
      driveSubsystem.manualDrive(driveSubsystem.angleError(neededAngle) * 0.4, -driveSubsystem.distanceError(neededDistance) * 0.5, 50, 35);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(driveSubsystem.distanceError(neededDistance)) < 3 && Math.abs(driveSubsystem.angleError(neededAngle)) < 1.5) {
      return true;
    }
    return false;
  
  }
}
