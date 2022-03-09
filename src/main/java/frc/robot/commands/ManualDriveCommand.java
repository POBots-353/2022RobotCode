// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.NewDriveSubsystem;

public class ManualDriveCommand extends CommandBase {
  private final NewDriveSubsystem driveSubsystem;
  private final Joystick driverStick;

  /** Creates a new ManualDriveCommand. */
  public ManualDriveCommand(NewDriveSubsystem drive) {
    driveSubsystem = drive;
    driverStick = RobotContainer.driverStick;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardDrive = -(DriveConstants.scaleY * (Math.pow(driverStick.getY(), 3))
        + (1 - DriveConstants.scaleY) * driverStick.getY());
    double turnDrive = (DriveConstants.scaleX * (Math.pow(driverStick.getX(), 3))
        + (1 - DriveConstants.scaleY) * driverStick.getX());

    driveSubsystem.drive(forwardDrive, turnDrive, DriveConstants.scaleFowd, DriveConstants.scaleTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
