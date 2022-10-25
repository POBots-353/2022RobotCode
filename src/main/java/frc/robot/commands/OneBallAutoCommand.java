// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ToggleArmCommand.PositionMode;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;


public class OneBallAutoCommand extends SequentialCommandGroup {
  public OneBallAutoCommand(DriveSubsystem drive, BallTransitSubsystem ballTransitSubsystem, ClimberSubsystem climbSubsystem) {
    //Make Sure to have a timeout after every Command, just incase the command doesn't end
    addCommands(

      new InstantCommand(()->ballTransitSubsystem.toggleShooter(), ballTransitSubsystem),
      new InstantCommand(()->ballTransitSubsystem.toggleShooter(), ballTransitSubsystem),
      new AutoDriveCommand(drive, -8.41 *  (100 / (6 * Math.PI))),
      new InstantCommand(()->climbSubsystem.toggleInnerArms(), climbSubsystem)
      //new InstantCommand(()->transitSubsystem.setArmAngle(PositionMode.goDown),tran
      //new InstantCommand(()->climbSubsystem.climberStop(), climbSubsystem),
      //Command list of wanted movement
       //Command list of wanted movement
      //new DumpBallCommand(transitSubsystem).withTimeout(1),
    //   new InstantCommand(()->ballTransitSubsystem.setArmAngle(PositionMode.goUp),ballTransitSubsystem),
    //   new WaitCommand(1),
    //   new AutoDriveCommand(drive, 8.41 * (40.44 / (6 * Math.PI))),
    //   new TurnToAngleCommand(drive, 174),
    //   // Drops the ball off first, turns 180 
    //   new ParallelRaceGroup(
    //     new AutoDriveCommand(drive, 8.41 *(97.8 / (6 * Math.PI))),
    //     new StartEndCommand(()->ballTransitSubsystem.inTake(), ()->ballTransitSubsystem.turnOffIntakeMotor(), ballTransitSubsystem)
    //     ),
    //   new TurnToAngleCommand(drive, 156).withTimeout(3), 
    //   new ParallelCommandGroup(
    //     new InstantCommand(()->ballTransitSubsystem.setArmAngle(PositionMode.goUp),ballTransitSubsystem), //We might want to manually drop intake
    //     new AutoDriveCommand(drive, 8.41 * (22.66 / (6 * Math.PI)))//was 100.44
    //   ),
    // //Turns 180 degrees and returns back to the drop with the ball 
    //   new StartEndCommand(()->ballTransitSubsystem.outTake(), ()->ballTransitSubsystem.turnOffIntakeMotor(), ballTransitSubsystem).withTimeout(0)
    
    
    

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
