// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AlignCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  
  // Use gyro declaration from above here

  // The gain for a simple P loop
  double kP = .4;
  double neededAngle = 0;

  /**
   * Turns robot to angle
   * @param subsystem
   * @param neededAngle input wanted angle
   */
  public AlignCommand(DriveSubsystem subsystem, double neededAngle) {
    driveSubsystem = subsystem;
    addRequirements(subsystem);
    this.neededAngle = neededAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    driveSubsystem.manualDrive(0.0, driveSubsystem.angleError(neededAngle) * kP, 50, 0.0);
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (driveSubsystem.angleError(neededAngle) < 2){
      return true;
    }
    return false;
  }
}
