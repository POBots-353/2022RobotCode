// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.DriveSubsystem;


public class OneBallAutoCommand extends SequentialCommandGroup {
  public OneBallAutoCommand(DriveSubsystem drive, BallTransitSubsystem ballTransitSubsystem) {
    addCommands(
      //new SwerveCommand(drive, -90, 70)
      new RunCommand(()->drive.manualDrive(0.5,0,1000,0), drive).withTimeout(15)
    );
  }
}
