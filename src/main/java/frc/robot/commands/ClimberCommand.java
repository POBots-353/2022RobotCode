// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {

	int climbingStage = 0;

	int timer = 0;
	int timerStarted = 0;
	int timerLength = 0;

	boolean repeatClimb = true;

	private final ClimberSubsystem climberSubsystem;

	/* Creates a new ClimberCommand. */
	public ClimberCommand(ClimberSubsystem climber) {
		climberSubsystem = climber;
		addRequirements(climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		updateTimer();

		double outerPosition = climberSubsystem.outerEncoder.getPosition();
		double innerPosition = climberSubsystem.innerEncoder.getPosition();

		climberSubsystem.setOuterArmsPosition(climberSubsystem.currentOuterReferencePoint);
		climberSubsystem.setInnerArmsPosition(climberSubsystem.currentInnerReferencePoint);

		switch (climbingStage) {
			/*
			 * This is the format for the comments:
			 * 
			 * 'Info on what we're doing at the end of the stage'
			 * 
			 * Outer Arms: 'What the position and retraction state will be after this stage'
			 * Inner Arms: 'What the position and retraction state will be after this stage'
			 * 
			 * 'Other info about what's happening after this stage'
			 */
			case 0:
				/*
				 * Outer Arms: Unknown position, retracted
				 * Inner Arms: Unknown position, retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterArmsPosition(Constants.behindBarSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 1;
				}
				break;
			case 1:
				/*
				 * At the end of this stage the outer arms are extended to reach the second bar
				 * 
				 * The robot will be sitting on the ground with the outer arms extended, the
				 * next step is to move the inner arms out of the way
				 * 
				 * Outer Arms: Behind the bar, extended
				 * Inner Arms: Unknown angle, retracted
				 */
				if (timerCompleted() && getPositionError(outerPosition, Constants.behindBarSetPoint) < 1.0) {
					climberSubsystem.toggleOuterArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 2;
				}
				break;
			case 2:
				/*
				 * At the end of this stage we move the inner arms out of the way (towards the
				 * behind bar set point
				 * 
				 * The robot will be on the ground with the outer arms vertical retracted and
				 * the inner arms will be forward retracted
				 * 
				 * Outer Arms: Behind the next bar, extended
				 * Inner Arms: Behind the next bar, retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setInnerArmsPosition(Constants.behindBarSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 3;
				}
				break;
			case 3:
				/*
				 * At the end of this stage we move the robot slightly forward to be aligned
				 * with the 2nd bar
				 * 
				 * The robot will be aligned with the 2nd bar with the outer arms extended
				 * outwards and the inner arms forward to be beneath the 3rd bar
				 * 
				 * Outer Arms: Aligned with the next bar, extended
				 * Inner Arms: Behind the next bar, retracted
				 */
				if (timerCompleted() && getPositionError(innerPosition, Constants.behindBarSetPoint) < 1.0) {
					climberSubsystem.setOuterArmsPosition(Constants.barAlignedSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 4;
				}
				break;
			case 4:
				/*
				 * At the end of this stage we retract the outer arms and we're on the 2nd bar
				 * 
				 * Outer Arms: On the bar (not perpendicular), retracted
				 * Inner Arms: Behind the next bar (the angle), retracted
				 */
				if (timerCompleted() && getPositionError(outerPosition, Constants.barAlignedSetPoint) < 1.0) {
					climberSubsystem.toggleOuterArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 5;
				}
				break;
			case 5:
				/*
				 * Outer Arms: On the next bar (perpendicular), retracted
				 * Inner Arms: Behind the next bar (the angle), retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterArmsPosition(Constants.verticalSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 6;
				}
				break;
			case 6:
				/*
				 * The inner arms get extended
				 * 
				 * Outer Arms: On the bar (perpendicular), retracted
				 * Inner Arms: Behind the next bar (the angle), extended
				 */
				if (timerCompleted()) {
					climberSubsystem.toggleInnerArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 7;
				}
				break;
			case 7:
				/*
				 * At the end of this stage we move the inner arms to be aligned with the bar
				 * 
				 * Outer Arms: Perpendicular to the ground, retracted
				 * Inner Arms: Perpendicular to the ground, extended
				 * 
				 * REPEAT TO HERE WHEN GOING TO 4TH BAR
				 */
				if (timerCompleted()) {
					resetTimer();
					climberSubsystem.setInnerArmsPosition(Constants.verticalSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 8;
				}
				break;
			case 8:
				/*
				 * At the end of this stage we retract the inner arms
				 * 
				 * Outer Arms: Perpendicular to the ground, retracted
				 * Inner Arms: Perpendicular to the ground, retracted
				 * 
				 * Both arms are now hung onto the 2nd/3rd bar
				 * 
				 * ***** END HERE UNTIL WE ARE READY TO GO TO THE 4TH BAR *****
				 */
				if (getPositionError(innerPosition, Constants.verticalSetPoint) < 1.0
						&& timerCompleted()) {
					climberSubsystem.toggleInnerArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 24;
				}
				break;
			case 9:
				/*
				 * At the end of this stage we extend the outer arms
				 * 
				 * Outer Arms: Perpendicular to the ground, extended
				 * Inner Arms: Perpendicular to the ground, retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.toggleOuterArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 10;
				}
				break;
			case 10:
				/*
				 * At the end of this stage the outer arms get moved to an angle between 90 and
				 * aligned with the next bar
				 * 
				 * Outer Arms: At an angle between 90 and aligned with the next bar, extended
				 * Inner Arms: Perpendicular to the ground, retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterArmsPosition(Constants.firstExtendSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 11;
				}
				break;
			case 11:
				/*
				 * At the end of this stage we extend the outer arms
				 * 
				 * Outer Arms: At an angle between 90 and aligned with the next bar, retracted
				 * Inner Arms: Perpendicular to the ground, retracted
				 */
				if (timerCompleted() && getPositionError(outerPosition, Constants.firstExtendSetPoint) < 1.0) {
					climberSubsystem.toggleOuterArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 12;
				}
				break;
			case 12:
				/*
				 * Outer Arms: Behind the next bar, retracted
				 * Inner Arms: Perpendicular to the ground, retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterArmsPosition(Constants.behindBarSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 13;
				}
				break;
			case 13:
				/*
				 * Outer Arms: Behind the next bar, extended
				 * Inner Arms: Perpendicular to the ground, retracted
				 */
				if (timerCompleted() && getPositionError(outerPosition, Constants.behindBarSetPoint) < 1.0) {
					climberSubsystem.toggleOuterArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 14;
				}
				break;
			case 14:
				/*
				 * At the end of this stage we move the outer arms behind the bar
				 * 
				 * Outer Arms: Aligned with the next bar, extended
				 * Inner Arms: Perpendicular to the ground, retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterArmsPosition(Constants.barAlignedSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 15;
				}
				break;
			case 15:
				/*
				 * At the end of this stage we retract the outer arms
				 * 
				 * Outer Arms: Aligned with the next bar, retracted
				 * Inner Arms: Perpendicular to the ground, retracted
				 * 
				 * In this step and the next stape we have to retract the arms and disable PID
				 * right before we make contact with the next bar to prevent the inner arms from
				 * going backwards
				 * 
				 * We are now hanging on 2 bars
				 */
				if (timerCompleted() && getPositionError(outerPosition, Constants.barAlignedSetPoint) < 1.0) {
					climberSubsystem.toggleOuterArms();
					startTimer(Constants.MLGWaterBucketClutchTime);
					climbingStage = 16;
				}
				break;
			case 16:
				/*
				 * MLG WATER BUCKET CLUTCH BABYYYYYYYYYYYYYYYYYYYYY
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterPID(false);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 17;
				}
				break;
			case 17:
				/*
				 * Outer Arms: Unknown Angle, retracted
				 * Inner Arms: Perpendicular to the ground, retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterArmsPosition(climberSubsystem.outerEncoder.getPosition());
					climberSubsystem.setOuterPID(true);
					climbingStage = 18;
				}
				break;
			case 18:
				/*
				 * At the end of this stage we release the inner arms
				 * 
				 * Outer Arms: Unknown angle, retracted
				 * Inner Arms: Perpendicular to the ground, extended
				 */
				if (timerCompleted()) {
					climberSubsystem.toggleInnerArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 19;
				}
				break;
			case 19:
				/*
				 * Outer Arms: On the next bar, retracted
				 * Inner Arms: At a position between 90 degrees and behind the next bar,
				 * extended
				 */
				if (timerCompleted()) {
					climberSubsystem.setInnerArmsPosition(Constants.firstExtendSetPoint);
					startTimer(Constants.pneumaticTimerDelay);
					climbingStage = 20;
				}
				break;
			case 20:
				/*
				 * Outer Arms: On the next bar, unknown angle, retracted
				 * Inner Arms: At a position between 90 degrees and behind the next bar,
				 * retracted
				 */
				if (timerCompleted() && getPositionError(innerPosition, Constants.firstExtendSetPoint) < 1.0) {
					climberSubsystem.toggleInnerArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 21;
				}
				break;
			case 21:
				/*
				 * This step will rotate the robot so that it is properly aligned
				 * 
				 * Outer Arms: 90 degrees, retracted
				 * Inner Arms: At a position between 90 degrees and behind the next bar,
				 * retracted
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterPID(true);
					climberSubsystem.setOuterArmsPosition(Constants.verticalSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 22;
				}
				break;
			case 22:
				/*
				 * Outer Arms: 90 degrees, retracted
				 * Inner Arms: Behind the next bar, retracted
				 */
				if (timerCompleted() && getPositionError(outerPosition, Constants.verticalSetPoint) < 1.0) {
					climberSubsystem.setInnerArmsPosition(Constants.behindBarSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 23;
				}
				break;
			case 23:
				/*
				 * Outer Arms: 90 degrees, retracted
				 * Inner Arms: Behind the next bar, extended
				 */
				if (timerCompleted() && getPositionError(innerPosition, Constants.behindBarSetPoint) < 1.0) {
					climberSubsystem.toggleInnerArms();
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 24;
				}
				break;
			case 24:
				/*
				 * Inner Arms get moved to repeat these steps (if necessary)
				 * 
				 * Outer Arms: Hung on 3rd/4th bar, retracted
				 * Inner Arms: Behind the next bar, extended
				 */
				if (timerCompleted()) {
					climberSubsystem.setInnerArmsPosition(Constants.verticalSetPoint);
					if (repeatClimb) {
						climbingStage = 7;
						repeatClimb = false;
					} else {
						climbingStage = 25;
					}
				}
				break;
			case 25:
				/* WE DID IT!!! */
				break;
			default:
				/* If this function gets called then we're screwed */
				break;
		}
	}

	// Called once{} the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		climberSubsystem.outerMotor.set(0);
		climberSubsystem.leftInnerMotor.set(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	public void updateTimer() {
		timer++;
	}

	public double getPositionError(double currentPosition, double expectedPosition) {

		return Math.abs(currentPosition - expectedPosition);
	}

	public void startTimer(int length) {
		timerStarted = timer;
		timerLength = length;
	}

	public boolean timerCompleted() {

		return (timer - timerStarted <= timerLength && timerLength != 0) ? resetTimer() : false;
	}

	public boolean resetTimer() {
		timerStarted = 0;
		timerLength = 0;
		return true;
	}

}
