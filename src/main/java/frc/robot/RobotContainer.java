// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.lang.management.ClassLoadingMXBean;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.commands.no;
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

	private final no no = new no(driveSubsystem, ballTransitSubsystem, climberSubsystem);
	private final OneBallAutoCommand OneBallAuto = new OneBallAutoCommand(driveSubsystem, ballTransitSubsystem, climberSubsystem);
	private final SimpleAuto simpleAuto = new SimpleAuto(driveSubsystem, ballTransitSubsystem, climberSubsystem);

	public static SendableChooser<Command> autoChooser = new SendableChooser<>();

	public static final Joystick driverStick = new Joystick(0);
	public static final Joystick operatorStick = new Joystick(1);

	//Compressor
	

	public RobotContainer() {
		// Auto Chooser on SmartDashboard
		autoChooser.setDefaultOption("no", no);
		//autoChooser.addOption("One Ball Auto", OneBallAuto);
		autoChooser.addOption("Simple Auto", simpleAuto);
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

		//Button 14
		//new JoystickButton(driverStick,14).whileHeld(()->climberSubsystem.moveClimberArms(0),climberSubsystem);

		new JoystickButton(operatorStick, 6).whenPressed(()->climberSubsystem.enableCompressor(), climberSubsystem);
		new JoystickButton(operatorStick, 9).whenPressed(()->climberSubsystem.disableCompressor(), climberSubsystem);

		// new PerpetualCommand(new RunCommand(()->driveSubsystem.disableCompressor(), driveSubsystem));
		//Position
		//new JoystickButton(driverStick, Buttons.outerClimbPosition).whileHeld(()->climberSubsystem.moveOuterArms(1),climberSubsystem);
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
						(DriveConstants.scaleY * (Math.pow(driverStick.getY(), 3))
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
		
		//Turn to Angle Other Side Buttons
		
		/*new JoystickButton(driverStick, Buttons.turn0Toggle2)
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
		 		() -> ballTransitSubsystem.inTake(), () -> ballTransitSubsystem.intakeOff(),
		 		ballTransitSubsystem));
		 new JoystickButton(operatorStick, Buttons.ballOutTake).whileHeld(new StartEndCommand(
		 		() -> ballTransitSubsystem.outTake(), () -> ballTransitSubsystem.turnOffIntakeMotor(),
		 		ballTransitSubsystem));
					
		// Toggles Arm
		// This locks the piston no matter which position
		new JoystickButton(operatorStick, Buttons.armToggle).whenPressed(new ToggleArmCommand(ballTransitSubsystem));
		new JoystickButton(operatorStick, Buttons.turnOffArm).whenPressed(new StartEndCommand(()->ballTransitSubsystem.turnOffArmMotor(), ()->ballTransitSubsystem.resetPosition(), ballTransitSubsystem).withTimeout(1.5));

		//Manual Climb
		new JoystickButton(operatorStick, Buttons.outerClimb).whenPressed(new StartEndCommand(()->climberSubsystem.toggleOuterArms(),()->climberSubsystem.climberOff(),climberSubsystem).withTimeout(1));
		//new JoystickButton(operatorStick, Buttons.outerClimb).whenPressed(()->climberSubsystem.toggleOuterArms(),climberSubsystem);
		new JoystickButton(operatorStick, Buttons.innerClimb).whenPressed(()->climberSubsystem.toggleInnerArms(),climberSubsystem);
		new JoystickButton(operatorStick, Buttons.climbFoward).whileHeld(()->climberSubsystem.moveForeward(),climberSubsystem);
		new JoystickButton(operatorStick, Buttons.climbBack).whileHeld(()->climberSubsystem.moveBackward(),climberSubsystem);
		new JoystickButton(operatorStick, Buttons.climbStop).whenPressed(()->climberSubsystem.disablePID(),climberSubsystem);
		
		/*
		 * new JoystickButton(operatorStick, Buttons.climberButton)
		 * .whileHeld(new ClimberCommand(climberSubsystem));
		 */

		new JoystickButton(operatorStick, Buttons.armUp).whileHeld(
				() -> ballTransitSubsystem.setArmAngle(PositionMode.goUp),
				ballTransitSubsystem);
		new JoystickButton(operatorStick, 1).whileHeld(() -> ballTransitSubsystem.setArmAngle(PositionMode.goUpHigher),
			ballTransitSubsystem);
		new JoystickButton(operatorStick, Buttons.armDown).whileHeld(
				() -> ballTransitSubsystem.setArmAngle(PositionMode.goDown),
				ballTransitSubsystem);
	}
	
	// Runs the auto command
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
