// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EyeBallCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final PhotonCamera eye = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  /** Creates a new EyeBallCommand. */
  public EyeBallCommand(DriveSubsystem drive) {
    driveSubsystem = drive;
    addRequirements(drive);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var eyeValues = eye.getLatestResult();
    if (eyeValues.hasTargets()) {

      double yaw = eyeValues.getBestTarget().getYaw();
      double yee = yaw;
      double pitch = eyeValues.getBestTarget().getPitch() + 18;
      double potch = pitch;
      driveSubsystem.manualDrive(yaw * 0.20, pitch * 0.3, 75, 200);
    }else{
      driveSubsystem.manualDrive(0, 0, 0, 0);
    }
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
