// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleAuto extends SequentialCommandGroup {
  /** Creates a new SimpleAuto. */
  public SimpleAuto(DriveSubsystem drive, BallTransitSubsystem transitSubsystem) {
    //This is to make an on the fly command
    addCommands(
      new InstantCommand(()->transitSubsystem.resetPosition(), transitSubsystem),
      new InstantCommand(()->transitSubsystem.releaseArm(), transitSubsystem),
      new WaitCommand(1.5),
      new StartEndCommand(()->transitSubsystem.outTake(), ()->transitSubsystem.turnOffIntakeMotor(), transitSubsystem).withTimeout(1)
      
      // new AutoDriveCommand(drive, -8.41 *  (23.125 / (6 * Math.PI))),
      // new TurnToAngleCommand(drive, 164),
      // new AutoDriveCommand(drive, 8.41 * (69 / (6 * Math.PI))),
      // new TurnToAngleCommand(drive, -60),
      // new AutoDriveCommand(drive, 8.41 * (107.1875 / (6 * Math.PI))),
      // new TurnToAngleCommand(drive, -53),
      // new AutoDriveCommand(drive, 8.41 * ( 100.44 / (6 * Math.PI))),
      // new TurnToAngleCommand(drive, 16),
      // new AutoDriveCommand(drive, 8.41 * (25.9 / (6 * Math.PI)))
    );
    }
}
