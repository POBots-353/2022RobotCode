// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class AlignCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  
  // Use gyro declaration from above here

  // The gain for a simple P loop
  double kP = .4;
  

  
  // navX iniialization

  double neededAngle1 = 0;
  /**
  * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignCommand(DriveSubsystem subsystem, double neededAngle) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    neededAngle1 = neededAngle;
    // Turns the robot to face the desired direction
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(driveSubsystem.angleError(neededAngle1)) > 2){
      driveSubsystem.manualDrive(0.0, driveSubsystem.angleError(neededAngle1) * kP, 35, 0.0);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
