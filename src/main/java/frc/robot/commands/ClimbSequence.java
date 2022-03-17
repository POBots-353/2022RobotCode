// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  ClimberSubsystem climberSubsystem;

  /**
   * Toggle the Outer Pneumatics
   * 
   * @return A {@link Command} to toggle the Outer Arms
   */
  Command ToggleOuterArms() {
    return new RunCommand(() -> climberSubsystem.toggleOuterArms(), climberSubsystem);
  }

  /**
   * Toggle the Inner Pneumatics
   * 
   * @return A {@link Command} to toggle the Inner Arms
   */
  Command ToggleInnerArms() {
    return new RunCommand(() -> climberSubsystem.toggleInnerArms(), climberSubsystem);
  }

  /**
   * Move the Outer Climbing Arms
   * 
   * @param position The Position to Move the Outer Arms to
   * @return A {@link Command} to move the Outer Arms
   */
  Command MoveOuterArms(double position) {
    return new RunCommand(() -> climberSubsystem.setOuterArmsPosition(position), climberSubsystem)
        .withInterrupt(climberSubsystem::outerMoveFinished);
  }

  /**
   * Delay the Climbing Cycle for a specified amount of time
   * 
   * @param seconds The Amount of Seconds to Delay
   * @return A {@link WaitCommand} to delay the Climb Sequence
   */
  Command Wait(double seconds) {
    return new WaitCommand(seconds);
  }

  Command DisablePID() {
    return new RunCommand(() -> climberSubsystem.disablePID(), climberSubsystem);
  }

  Command EnablePID(double position) {
    return new RunCommand(() -> {
      climberSubsystem.setOuterPID(true);
      climberSubsystem.setOuterArmsPosition(position);
    }, climberSubsystem);
  }

  /** Creates a new ClimbSequence. */
  public ClimbSequence(ClimberSubsystem climber) {
    climberSubsystem = climber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        /*
         * Prerequisites:
         * Outer: Retracted; On mid bar
         * Inner: Retracted; Below mid bar
         */

        /*
         * Outer: Retracted; On the mid bar
         * Inner: Extended; Above the mid bar
         */
        ToggleInnerArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; Above the mid bar
         * Inner: Retracted; On the mid bar
         */
        ToggleInnerArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; Behind the next bar
         * Inner: Retracted; On the mid bar
         */
        MoveOuterArms(Constants.behindBarSetPoint),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended; behind the high
         * Inner: Retracted; on the mid bar
         */
        ToggleOuterArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended; Aligned with the high
         * Inner: Retracted; On the mid bar
         */
        MoveOuterArms(Constants.barAlignedSetPoint),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; On the High Bar
         * Inner: Retracted; In the Air
         */
        ToggleOuterArms(),
        Wait(Constants.MLGWaterBucketClutchSeconds),
        /**
         * We disable PID at the right time to let the Robot move naturally
         *
         * Outer: Retracted; On the High Bar
         * Inner: Retracted; In the Air
         */
        DisablePID(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; At the original start position
         * Inner: Retracted; At the original start position
         */
        EnablePID(climberSubsystem.outerEncoder.getPosition()),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; On the mid bar
         * Inner: Extended; Above the mid bar
         */
        ToggleInnerArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; Above the mid bar
         * Inner: Retracted; On the mid bar
         */
        ToggleInnerArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; Behind the next bar
         * Inner: Retracted; On the mid bar
         */
        MoveOuterArms(Constants.behindBarSetPoint),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended; Behind the high bar
         * Inner: Retracted; On the mid bar
         */
        ToggleOuterArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended; Aligned with the high bar
         * Inner: Retracted; On the mid bar
         */
        MoveOuterArms(Constants.barAlignedSetPoint),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; On the High Bar
         * Inner: Retracted; In the Air
         */
        ToggleOuterArms(),
        Wait(Constants.MLGWaterBucketClutchSeconds),
        /**
         * We disable PID at the right time to let the Robot move naturally
         *
         * Outer: Retracted; On the High Bar
         * Inner: Retracted; In the Air
         */
        DisablePID(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; At the original start position
         * Inner: Retracted; At the original start position
         */
        EnablePID(climberSubsystem.outerEncoder.getPosition()),
        Wait(Constants.secondsDelayBetweenSteps));
  }
}