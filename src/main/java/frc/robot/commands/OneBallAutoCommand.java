// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.DriveSubsystem;


public class OneBallAutoCommand extends SequentialCommandGroup {
  public OneBallAutoCommand(DriveSubsystem drive, BallTransitSubsystem ballTransitSubsystem) {
    //Make Sure to have a timeout after every Command, just incase the command doesn't end
    addCommands(
      //Command list of wanted movement
       //Command list of wanted movement
      //new DumpBallCommand(transitSubsystem).withTimeout(1),
      new AutoDriveCommand(drive, 8.41 * (34.6875 / (6 * Math.PI))),
      new TurnToAngleCommand(drive, 180),
      // Drops the ball off first, turns 180 
      //new IntakeBallCommand(transitSubsystem),
      //new ParallelRaceGroup(
        new AutoDriveCommand(drive, 8.41 *(115.625 / (6 * Math.PI))),
        //new StartEndCommand(() -> ballTransitSubsystem.toggleIntake(true),
          //      () -> ballTransitSubsystem.toggleIntake(false),
            //          ballTransitSubsystem)
        //),
        new TurnToAngleCommand(drive, 180), 
        new AutoDriveCommand(drive, 8.41 *(115.625 / (6 * Math.PI)))
    //Turns 180 degrees and returns back to the drop with the ball 
     /* new StartEndCommand(() -> ballTransitSubsystem.toggleShooter(true),
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
        new DumpBallCommand(transitSubsystem).withTimeout(1)*/
    );
  }
}
