// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoNavCommand extends SequentialCommandGroup {
  /**
   * This is where most of the auto code should go.
   * It should be built by using commands that are running seqentially
   * to prevent code from being repeated
   * @param drive
   * @param transitSubsystem
   */
  public AutoNavCommand(DriveSubsystem drive, BallTransitSubsystem transitSubsystem) {
    addCommands(
        //Test
        new SetDistanceCommand(drive, 80),

        new TurnToAngleCommand(drive, -91),

        new AutoDriveCommand(drive, 70),

        new TurnToAngleCommand(drive, 91),

        new AutoDriveCommand(drive, 70),

        new TurnToAngleCommand(drive, 0),

        new SetDistanceCommand(drive, 90),

        new TurnToAngleCommand(drive, -91),

        new AutoDriveCommand(drive, 150)
        // new AutoDriveCommand(drive, -35)
       // new AutoDriveCommand(drive, 50)*/
        // Turns to specified angle
        /*new AlignCommand(drive, 50), // Enter wanted angle

        new ParallelRaceGroup(
            new AutoDriveCommand(drive,50), // Drives the robot to specifed distance,
                                                                             // stops after two seconds
            new IntakeBallCommand(transitSubsystem).withTimeout(2)),

        new AutoDriveCommand(drive,50),

        new DumpBallCommand(transitSubsystem).withTimeout(1)*/);
        
  }
}
