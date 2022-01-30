// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoNavCommand extends SequentialCommandGroup {
  /** Creates a new AutoNavCommand. */
  public AutoNavCommand(DriveSubsystem drive, BallTransitSubsystem transitSubsystem) {
    addCommands(
        //Test
        new RunCommand(()->drive.autoDrive(50), drive)
        // Turns to specified angle
        /*new AlignCommand(drive, 50), // Enter wanted angle

        new ParallelCommandGroup(
            new RunCommand(() -> drive.autoDrive(50), drive).withTimeout(2), // Drives the robot to specifed distance,
                                                                             // stops after two seconds
            new IntakeBallCommand(transitSubsystem).withTimeout(2)),

        new RunCommand(() -> drive.autoDrive(50), drive).withTimeout(2),

        new DumpBallCommand(transitSubsystem).withTimeout(1)*/);
        
  }
}
