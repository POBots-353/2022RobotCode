// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.EyeBallCommand;
import frc.robot.commands.OneBallAutoCommand;
import frc.robot.commands.SetDistanceCommand;
import frc.robot.commands.SimpleAuto;
import frc.robot.commands.ToggleArmCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TwoBallAutoCommand;
import frc.robot.commands.ToggleArmCommand.PositionMode;
import frc.robot.subsystems.BallTransitSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();

	private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
	private final BallTransitSubsystem ballTransitSubsystem = new BallTransitSubsystem();

	private final TwoBallAutoCommand TwoBallAuto = new TwoBallAutoCommand(driveSubsystem, ballTransitSubsystem);
	private final OneBallAutoCommand OneBallAuto = new OneBallAutoCommand(driveSubsystem, ballTransitSubsystem);
	private final SimpleAuto simpleAuto = new SimpleAuto(driveSubsystem, ballTransitSubsystem);

	public static SendableChooser<Command> autoChooser = new SendableChooser<>();

	public static final Joystick driverStick = new Joystick(0);
	public static final Joystick operatorStick = new Joystick(1);

	public RobotContainer() {
		// Auto Chooser on SmartDashboard
		autoChooser.setDefaultOption("Two Ball Auto", simpleAuto);
		autoChooser.addOption("One Ball Auto", OneBallAuto);
		autoChooser.addOption("Simple Auto", TwoBallAuto);
		SmartDashboard.putData(autoChooser);
		
		// Default Drive
		driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.manualDrive(
				(DriveConstants.scaleX * (Math.pow(driverStick.getX(), 3)) +
						(1 - DriveConstants.scaleY) * driverStick.getX()),
				-(DriveConstants.scaleY *
						(Math.pow(driverStick.getY(), 3))
						+ (1 - DriveConstants.scaleY) *
								driverStick.getY()),
				DriveConstants.scaleTurn, DriveConstants.scaleFowd), driveSubsystem));
	
		// newDriveSubsystem.setDefaultCommand(new ManualDriveCommand(newDriveSubsystem));

		configureButtonBindings();
		testButtons();
	}

	private void testButtons(){
		// bnew InstantCommand(()->climberSubsystem.moveOuterArms(0),climberSubsystem);
		
		//new JoystickButton(driverStick, Buttons.innerArmToggle).whenPressed(()->climberSubsystem.toggleInnerArms(),climberSubsystem);
		//Vertical
		//new JoystickButton(driverStick, Buttons.outerClimbVertical).whileHeld(()->climberSubsystem.moveOuterArms(1),climberSubsystem);
		//Position
		//new JoystickButton(driverStick, Buttons.outerClimbPosition).whileHeld(()->climberSubsystem.moveOuterArms(1),climberSubsystem);
		//new JoystickButton(driverStick, Buttons.outerClimbVariable).whileHeld(()->climberSubsystem.moveOuterArms(operatorStick.getY()), climberSubsystem);
	}
 
	/*
	 * The buttons will be shecduled before the default Command and
	 * if the buttons that are activated have the same subsystem requirements as the
	 * default
	 * command,
	 * the defalut command will not run
	 */
	/*
	 * Also, be careful about brownouts (if to many things are running at once it
	 * will cause it), especailly if your running PID on motors and neumatics, so
	 * make sure it
	 * gets disabled after use execept drive
	 */
	private void configureButtonBindings() {
		/* DRIVE BUTTONS */
		// Inverse drive
		new JoystickButton(driverStick, Buttons.inverseControl)
				.whileHeld(new RunCommand(() -> driveSubsystem.manualDrive(
						-(DriveConstants.scaleX * (Math.pow(driverStick.getX(), 3))
								+ (1 - DriveConstants.scaleY) * driverStick.getX()),
						-(DriveConstants.scaleY * (Math.pow(driverStick.getY(), 3))
								+ (1 - DriveConstants.scaleY) * driverStick.getY()),
						DriveConstants.scaleTurn, DriveConstants.scaleFowd), driveSubsystem));

		// Sets Distance between wall and sensor
		new JoystickButton(driverStick, Buttons.setDistanceButton)
				.whileHeld(new SetDistanceCommand(driveSubsystem, 196));

		// Turn to Angle Buttons
		new JoystickButton(driverStick, Buttons.turn0Toggle)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle0));
		new JoystickButton(driverStick, Buttons.turn90Toggle)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle90));
		new JoystickButton(driverStick, Buttons.turnNegative90Toggle)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngleNegative90));
		new JoystickButton(driverStick, Buttons.turn180Toggle)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle180));
		new JoystickButton(driverStick, Buttons.turnToCilmb)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededCilmbAngle));

		//Turn to Angle Other Side Buttons
		/*
		new JoystickButton(driverStick, Buttons.turn0Toggle2)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle0));
		new JoystickButton(driverStick, Buttons.turn90Toggle2)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle90));
		new JoystickButton(driverStick, Buttons.turnNegative90Toggle2)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngleNegative90));
		new JoystickButton(driverStick, Buttons.turn180Toggle2)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle180));
				*/

		/* OPERATOR BUTTONS */
		// Auto align to ball command
		new JoystickButton(operatorStick, Buttons.eyeballLeftButton)
				.whileHeld(new EyeBallCommand(driveSubsystem, Constants.yawLeftBias));
		new JoystickButton(operatorStick, Buttons.eyeballRightButton)
				.whileHeld(new EyeBallCommand(driveSubsystem, Constants.yawRightBias));

		// Intake in and out
		 new JoystickButton(operatorStick, Buttons.ballIntake).whileHeld(new StartEndCommand(
		 		() -> ballTransitSubsystem.inTake(), () -> ballTransitSubsystem.turnOffIntakeMotor(),
		 		ballTransitSubsystem));
		 new JoystickButton(operatorStick, Buttons.ballOutTake).whileHeld(new StartEndCommand(
		 		() -> ballTransitSubsystem.outTake(), () -> ballTransitSubsystem.turnOffIntakeMotor(),
		 		ballTransitSubsystem));

		//Manual Climb
		 new JoystickButton(operatorStick, Buttons.outerArmToggle).whenPressed(()->climberSubsystem.toggleOuterArms(),climberSubsystem);
					
		// Toggles Arm
		// This locks the piston no matter which position
		new JoystickButton(operatorStick, Buttons.armToggle).whenPressed(new ToggleArmCommand(ballTransitSubsystem));
		
		
		/*
		 * new JoystickButon(operatorStick,
		 * Buttons.manualClimb).whileHeld(()->climberSubsystem.piston.toggle());
		 */

		/*
		 * new JoystickButton(operatorStick, Buttons.climberButton)
		 * .whileHeld(new ClimberCommand(climberSubsystem));
		 */

		// TESTING CODE
		new JoystickButton(operatorStick, Buttons.armUp).whileHeld(
				() -> ballTransitSubsystem.setArmAngle(PositionMode.goUp),
				ballTransitSubsystem);
		new JoystickButton(operatorStick, Buttons.armRelease).whileHeld(
					() -> ballTransitSubsystem.setArmAngle(PositionMode.goUpHigher),
					ballTransitSubsystem);
		new JoystickButton(operatorStick, Buttons.armDown).whileHeld(
				() -> ballTransitSubsystem.setArmAngle(PositionMode.goDown),
				ballTransitSubsystem);
		// new JoystickButton(operatorStick,
		// 4).whileHeld(newRunCommand(()->ballTransitSubsystem.togglePiston()));
	}

	// Runs the auto command
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
