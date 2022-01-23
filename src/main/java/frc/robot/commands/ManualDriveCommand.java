// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Buttons;
import frc.robot.subsystems.DriveSubsystem;

public class ManualDriveCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  public ManualDriveCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = drive;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled. 
  @Override
  public void execute() {
    boolean inverseDriving = RobotContainer.driverStick.getRawButton(Buttons.inverseControl);
    double move = RobotContainer.driverStick.getY();
    double turn = RobotContainer.driverStick.getX();
    double turnConstant = Constants.scaleTurn;
    double velocity = driveSubsystem.leftBackEncoder.getVelocity();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      driveSubsystem.manualDrive(0, 0, 0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
