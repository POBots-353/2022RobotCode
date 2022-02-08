// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TwoBallAutoCommand extends SequentialCommandGroup {
  /**
   * This is where most of the auto code should go.
   * It should be built by using commands that are running seqentially
   * to prevent code from being repeated
   * @param drive
   * @param transitSubsystem
   */
  public TwoBallAutoCommand(DriveSubsystem drive, BallTransitSubsystem transitSubsystem) {
    //Make Sure to have a timeout after every Command, just incase the command doesn't end
    addCommands(
      //Command list of wanted movement
      /*new DumpBallCommand(transitSubsystem).withTimeout(1),
      new AutoDriveCommand(drive, -20),
      new TurnToAngleCommand(drive, 180),
      new IntakeBallCommand(transitSubsystem),
      new ParallelRaceGroup(
        new AutoDriveCommand(drive, 10),
        new StartEndCommand(() -> ballTransitSubsystem.toggleIntake(true),
                () -> ballTransitSubsystem.toggleIntake(false),
                      ballTransitSubsystem)
        ),
      new TurnToAngleCommand(drive, -90),
      new ParallelRaceGroup(
        new AutoDriveCommand(drive, 5),
        new StartEndCommand(() -> ballTransitSubsystem.toggleIntake(true),
                () -> ballTransitSubsystem.toggleIntake(false),
                      ballTransitSubsystem)
        ),
      new TurnToAngleCommand(drive, 0),
      //new IntakeBallCommand(transitSubsystem).withInterrupt(transitSubsystem::getUpPiston),
      new AutoDriveCommand(drive, 10),
      //new TurnToAngleCommand(drive, 0),
      //new AutoDriveCommand(drive, 10),
      new DumpBallCommand(transitSubsystem),
      new StartEndCommand(() -> ballTransitSubsystem.toggleShooter(true),
          () -> ballTransitSubsystem.toggleShooter(false),
               ballTransitSubsystem)
      */

        //Tests
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
