// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleAuto extends SequentialCommandGroup {
  /** Creates a new SimpleAuto. */
  public SimpleAuto(DriveSubsystem drive, BallTransitSubsystem transitSubsystem) {
    //This is to make an on the fly command
    addCommands(
      new InstantCommand(()->transitSubsystem.releaseArm(), transitSubsystem),
      new AutoDriveCommand(drive, 30)
    );
  }
}
