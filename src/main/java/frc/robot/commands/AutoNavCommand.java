// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
      //new AutoDriveCommand(drive, -50),
      //new RunCommand(()-> drive.resetGyro(), drive).withTimeout(2)
      //new AutoDriveCommand(drive, -50)
      new DumpBallCommand(transitSubsystem).withTimeout(1),
      new AutoDriveCommand(drive, -20),
      new TurnToAngleCommand(drive, 180),
      new IntakeBallCommand(transitSubsystem).withInterrupt(transitSubsystem::getDownPiston),
      new ParallelRaceGroup(
        new AutoDriveCommand(drive, 10),
        new IntakeBallCommand(transitSubsystem)
        ),
      new TurnToAngleCommand(drive, -90),
      new ParallelRaceGroup(
        new AutoDriveCommand(drive, 5),
        new IntakeBallCommand(transitSubsystem)
        ),
      new TurnToAngleCommand(drive, 0),
      //new IntakeBallCommand(transitSubsystem).withInterrupt(transitSubsystem::getUpPiston),
      new AutoDriveCommand(drive, 10),
      //new TurnToAngleCommand(drive, 0),
      //new AutoDriveCommand(drive, 10),
      new DumpBallCommand(transitSubsystem).withTimeout(4)

        //Test
        /*new SetDistanceCommand(drive, 80),

        new TurnToAngleCommand(drive, -91),

        new AutoDriveCommand(drive, 50),

        new TurnToAngleCommand(drive, 90),

        new AutoDriveCommand(drive, 50),

        new TurnToAngleCommand(drive, 0),

        new AutoDriveCommand(drive, -25).withInterrupt(transitSubsystem::getDownPiston)*/
        
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
