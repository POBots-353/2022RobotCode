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

		climberSubsystem.setOuterArmsPosition(climberSubsystem.currentOuterReferencePoint);

		switch (climbingStage) {
			/*
			 * INNER AND OUTER ARMS ARE ON THE 2ND BAR BEFORE CLIMBING STARTS
			 * Outer Position: Vertical, retracted
			 */
			case 0:
				break;
			case 1:
				break;
			case 2:
				/*
				 * Outer Arms: Vertical, Extended
				 */
				climberSubsystem.toggleOuterArms();
				startTimer(Constants.timerDelayBetweenSteps);
				climbingStage = 3;
				break;
			case 3:
				/*
				 * Outer Arms: At some position between 90 and at the bar, Extended
				 */
				if (timerCompleted()) {
					climberSubsystem.setOuterArmsPosition(Constants.firstExtendSetPoint);
					startTimer(Constants.timerDelayBetweenSteps);
					climbingStage = 4;
				}
				break;
			case 4:
				if (timerCompleted() && getPositionError(outerPosition, Constants.firstExtendSetPoint) < 1.0) {

				}
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
