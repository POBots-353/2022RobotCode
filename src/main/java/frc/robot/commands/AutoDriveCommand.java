// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {
  /**
   * This class is not meant to add any auto code
   * it was built so that it can end after a condition is reached
   * and make it easier to code autoNav
   */

  private final DriveSubsystem driveSubsystem;
  private double displacement;

  public AutoDriveCommand(DriveSubsystem drive, double displacement) {
    driveSubsystem = drive;
    this.displacement = displacement;
    addRequirements(driveSubsystem);
  }

  // Resets Encoders once the method runs to prevent the robot from doing other
  // movements
  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  //Moves Robot to set position
  @Override
  public void execute() {
    driveSubsystem.autoDrive(displacement);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      driveSubsystem.resetEncoders();
    }
  }

  // Returns true when the point is reached and resets the encoders
  @Override
  public boolean isFinished() {
    return driveSubsystem.pointReached(displacement);
  }
}
