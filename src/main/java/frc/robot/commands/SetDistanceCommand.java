// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class SetDistanceCommand extends CommandBase {
  /** Creates a new SetDistanceCommand. */
  //public final AnalogInput input = new AnalogInput(0);
  private final DriveSubsystem driveSubsystem;
  public SetDistanceCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  @Override
  public void execute() {
      // Called every time the scheduler runs while the command is scheduled.
    if (((driveSubsystem.ultrasonic.getValue() / 29) / 2) * 2.54 > 7){

  }
    //SmartDashboard.putNumber("UltraSonic", input.getValue() * 0.125);
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
