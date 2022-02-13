// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EyeBallCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final PhotonCamera eye = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  double yaw; // The x of the camera view
  double pitch; // The y of the camera view

  public EyeBallCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets the values from this object
    PhotonPipelineResult eyeValues = eye.getLatestResult();

    if (eyeValues.hasTargets()) {
      //Add or subtract to the yaw or pitch to get to a dersired location on the camera
      yaw = eyeValues.getBestTarget().getYaw();
      pitch = eyeValues.getBestTarget().getPitch() + 18;
      driveSubsystem.manualDrive(yaw * 0.20, pitch * 0.3, 75, 200);
    } else if (yaw >= 5 || pitch >= 5) { // This is to prevent the robot from stoping when tracking is flickering
      driveSubsystem.manualDrive(yaw * 0.20, pitch * 0.3, 75, 200);
    } else {
      driveSubsystem.manualDrive(0, 0, 0, 0);
    }
  }

  //I need to add an end condition
  @Override
  public void end(boolean interrupted) {
  }

  //I need to add an end condition
  @Override
  public boolean isFinished() {
    return false;
  }
}
