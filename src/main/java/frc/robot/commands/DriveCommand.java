// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  public final DriveSubsystem driveSubsystem;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem system) {
    driveSubsystem = system;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double moveSpeed = RobotContainer.driverController.getY();
    double rotateSpeed = RobotContainer.driverController.getX();

    driveSubsystem.arcadeDrive(moveSpeed, rotateSpeed, getScaleConstant());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      driveSubsystem.arcadeDrive(0, 0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public double getScaleConstant() {
    double scale = 0.0;
    boolean turbo = RobotContainer.driverController.getRawButton(Constants.turboButtonNumber);
    boolean slow = RobotContainer.driverController.getRawButton(Constants.slowButtonNumber);

    if(turbo) {
      scale = Constants.turboScale;
    } else if(slow) {
      scale = Constants.slowScale;
    } else {
      scale = Constants.driverScale;
    }
    return scale;
  }
}
