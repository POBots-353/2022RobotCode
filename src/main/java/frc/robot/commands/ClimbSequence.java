// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  public ClimbSequence(ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        /*
         * Outer: Extended
         * Inner: Retracted
         */
        new RunCommand(() -> climberSubsystem.toggleOuterArms(), climberSubsystem),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended
         * Inner: Extended
         */
        new RunCommand(() -> climberSubsystem.toggleInnerArms(), climberSubsystem),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted
         * Inner: Extended
         */
        new RunCommand(() -> climberSubsystem.toggleOuterArms(), climberSubsystem),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted
         * Inner: Retracted
         */
        new RunCommand(() -> climberSubsystem.toggleInnerArms(), climberSubsystem),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended, Position Between 90 and Next Bar
         * Inner: Retracted
         */
        new RunCommand(() -> climberSubsystem.setOuterArmsPosition(Constants.firstExtendSetPoint), climberSubsystem)
            .withInterrupt(climberSubsystem::outerMoveFinished),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted, Behind the next bar
         * Inner: Retracted
         */
        new RunCommand(() -> climberSubsystem.setOuterArmsPosition(Constants.behindBarSetPoint), climberSubsystem)
            .withInterrupt(climberSubsystem::outerMoveFinished),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended, Behind the next bar
         * Inner: Retracted
         */
        new RunCommand(() -> climberSubsystem.toggleOuterArms()),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended, Aligned with the next bar
         * Inner: Retracted
         */
        new RunCommand(() -> climberSubsystem.setOuterArmsPosition(Constants.barAlignedSetPoint))
            .withInterrupt(climberSubsystem::outerMoveFinished),
        new WaitCommand(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted, On the next Bar (PID DISABLED)
         * Inner: Retracted
         */
        new RunCommand(() -> climberSubsystem.toggleOuterArms(), climberSubsystem),
        new WaitCommand(Constants.MLGWaterBucketClutchSeconds),
        /*
         * MLG WATER BUCKET CLUTCH
         * We disable PID at the right time to prevent stuff from breaking (probably
         * gonna break anyway but it won't be my fault)
         */
        new RunCommand(() -> {
          climberSubsystem.setOuterPID(false);
          climberSubsystem.disablePID();
        }, climberSubsystem),
        new WaitCommand(Constants.secondsDelayBetweenSteps));
  }
}