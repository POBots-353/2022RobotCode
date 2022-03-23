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
    //Use ToggleArm Command because at the end of Auto the robot will be disable and the arm will drop
    addCommands(
      //Command list of wanted movement
      new InstantCommand(()->transitSubsystem.setArmAngle(PositionMode.goUp),transitSubsystem),
      new WaitCommand(1),
      new StartEndCommand(()->transitSubsystem.outTake(), ()->transitSubsystem.turnOffIntakeMotor(), transitSubsystem).withTimeout(1),
      new AutoDriveCommand(drive, -8.41 *  (23.125 / (6 * Math.PI))),
      new InstantCommand(()->transitSubsystem.setArmAngle(PositionMode.goDown),transitSubsystem), //We might want to manually drop intake
      new TurnToAngleCommand(drive, 164),//was 175 from testing in library
      new ParallelRaceGroup(
        new AutoDriveCommand(drive, 8.41 * (111.78 / (6 * Math.PI))),//was 69
         new StartEndCommand(()->transitSubsystem.inTake(), ()->transitSubsystem.turnOffIntakeMotor(), transitSubsystem)
       ),
      new TurnToAngleCommand(drive, -60),//was -80
      new ParallelRaceGroup(
        new AutoDriveCommand(drive, 8.41 * (127.1875 / (6 * Math.PI))),//was 107.1875
         new StartEndCommand(()->transitSubsystem.inTake(), ()->transitSubsystem.turnOffIntakeMotor(), transitSubsystem)
       ),
      
      new TurnToAngleCommand(drive,53),//was 40
      new ParallelCommandGroup(
        new InstantCommand(()->transitSubsystem.setArmAngle(PositionMode.goUp),transitSubsystem), //We might want to manually drop intake
        new AutoDriveCommand(drive, 8.41 * (118.44 / (6 * Math.PI)))//was 100.44
      ),
      
      
      new TurnToAngleCommand(drive, 16), //was 1
      new AutoDriveCommand(drive, 8.41 * (25.9 / (6 * Math.PI))),//was 30
      new StartEndCommand(()->transitSubsystem.outTake(), ()->transitSubsystem.turnOffIntakeMotor(), transitSubsystem).withTimeout(1)
      
    );
        
  }
}
