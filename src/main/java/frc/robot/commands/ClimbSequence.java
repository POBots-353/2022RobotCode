// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
   * @return An {@link InstantCommand} to toggle the Outer Arms
   */
  InstantCommand ToggleOuterArms() {
    return new InstantCommand(() -> climberSubsystem.toggleOuterArms(), climberSubsystem);
  }

  /**
   * Toggle the Inner Pneumatics
   * 
   * @return An {@link InstantCommand} to toggle the Inner Arms
   */
  InstantCommand ToggleInnerArms() {
    return new InstantCommand(() -> climberSubsystem.toggleInnerArms(), climberSubsystem);
  }

  /**
   * Move the Outer Climbing Arms
   * 
   * @param position The Position to Move the Outer Arms to
   * @return A {@link ParallelRaceGroup} to move the Outer Arms
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
  WaitCommand Wait(double seconds) {
    return new WaitCommand(seconds);
  }

  /**
   * Disable PID for the Climber Motors
   * 
   * @return An {@link InstantCommand} to disable PID
   */
  InstantCommand DisablePID() {
    return new InstantCommand(() -> climberSubsystem.disablePID(), climberSubsystem);
  }

  /**
   * Enable PID for the Climber Motors and
   * 
   * @param position The position to move the Motors to after enabling PID
   * @return An {@link InstantCommand} to enable PID and move to a position
   */
  InstantCommand EnablePID(double position) {
    return new InstantCommand(() -> climberSubsystem.enablePID(position), climberSubsystem);
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
         * We disable PID at the right time to let the Robot move naturally and not
         * destroy the elctric board or break the motor
         *
         * Outer: Retracted; On the High Bar
         * Inner: Retracted; In the Air
         * 
         * The Inner arms will let go of the bar on its own
         * 
         * We disable for an extended period to allow the robot to naturally fall to its
         * starting point
         */
        DisablePID(),
        Wait(Constants.secondsDelayAfterGettingToNewBar),
        /*
         * Outer: Retracted; At the original start position
         * Inner: Retracted; At the original start position
         */
        EnablePID(climberSubsystem.outerEncoder.getPosition()),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; On the high bar
         * Inner: Extended; Above the high bar
         */
        ToggleInnerArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; Above the high bar
         * Inner: Retracted; On the high bar
         */
        ToggleInnerArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; Behind the traversal bar
         * Inner: Retracted; On the high bar
         */
        MoveOuterArms(Constants.behindBarSetPoint),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended; Behind the traversal bar
         * Inner: Retracted; On the high bar
         */
        ToggleOuterArms(),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Extended; Aligned with the traversal bar
         * Inner: Retracted; On the high bar
         */
        MoveOuterArms(Constants.barAlignedSetPoint),
        Wait(Constants.secondsDelayBetweenSteps),
        /*
         * Outer: Retracted; On the traversal Bar
         * Inner: Retracted; In the Air
         */
        ToggleOuterArms(),
        Wait(Constants.MLGWaterBucketClutchSeconds),
        /**
         * We disable PID at the right time to let the Robot move naturally and not
         * destroy the electrical board or break the motor
         * 
         * The Inner arms will let go of the bar on its own
         *
         * Outer: Retracted; On the Traversal Bar
         * Inner: Retracted; In the Air
         * 
         * We disable for an extended period to allow the robot to naturally fall to its
         * starting point
         */
        DisablePID(),
        Wait(Constants.secondsDelayAfterGettingToNewBar),
        /*
         * Outer: Retracted; At the original start position
         * Inner: Retracted; At the original start position
         */
        EnablePID(climberSubsystem.outerEncoder.getPosition()));
  }
}