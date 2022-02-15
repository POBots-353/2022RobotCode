// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.ClimberCommand;
import frc.robot.commands.EyeBallCommand;
import frc.robot.commands.OneBallAutoCommand;
import frc.robot.commands.ToggleArmCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TwoBallAutoCommand;
import frc.robot.commands.ToggleArmCommand.PositionMode;
import frc.robot.subsystems.BallTransitSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;
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

	// private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
	private final BallTransitSubsystem ballTransitSubsystem = new BallTransitSubsystem();
	// BallTransitSubsystem();
	private final TwoBallAutoCommand TwoBallAuto = new TwoBallAutoCommand(driveSubsystem, ballTransitSubsystem);
	public final OneBallAutoCommand OneBallAuto = new OneBallAutoCommand(driveSubsystem, ballTransitSubsystem);
	public static SendableChooser<Command> m_chooser = new SendableChooser<>();
	// Put the chooser on the dashboard

	public static final Joystick driverStick = new Joystick(1);
	public static final Joystick operatorStick = new Joystick(0);

	public RobotContainer() {
		m_chooser.setDefaultOption("Two Ball Auto", TwoBallAuto);
		m_chooser.addOption("One Ball Auto", OneBallAuto);
		SmartDashboard.putData(m_chooser);
		// Configure the button bindings

		driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.manualDrive(
				(DriveConstants.scaleX * (Math.pow(driverStick.getX(), 3)) +
						(1 - DriveConstants.scaleY) * driverStick.getX()),
				-(DriveConstants.scaleY *
						(Math.pow(driverStick.getY(), 3))
						+ (1 - DriveConstants.scaleY) *
								driverStick.getY()),
				DriveConstants.scaleTurn, DriveConstants.scaleFowd), driveSubsystem));
		configureButtonBindings();
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
		new JoystickButton(driverStick, Buttons.inverseControl)
				.whileHeld(new RunCommand(() -> driveSubsystem.manualDrive(
						-(DriveConstants.scaleY * (Math.pow(driverStick.getY(), 3))
								+ (1 - DriveConstants.scaleY) * driverStick.getY()),
						-(DriveConstants.scaleX * (Math.pow(driverStick.getX(), 3))
								+ (1 - DriveConstants.scaleY) * driverStick.getX()),
						DriveConstants.scaleTurn, DriveConstants.scaleFowd), driveSubsystem));
		// new JoystickButton(operatorStick, 1).whileHeld(new
		// SetDistanceCommand(driveSubsystem, 196));
		// Turn to Angle Buttons
		/*new JoystickButton(driverStick, Buttons.turn45Toggle)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle45));
		new JoystickButton(driverStick, Buttons.turn90Toggle)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle90));
		new JoystickButton(driverStick, Buttons.turn180Toggle)
				.whileHeld(new TurnToAngleCommand(driveSubsystem, Constants.neededAngle180));*/
		/*
		 * new JoystickButton(operatorStick, Buttons.climberButton)
		 * .whileHeld(new ClimberCommand(climberSubsystem));
		 */
		// new JoystickButton(operatorStick, Buttons.toggleArm).whenPressed(new ToggleArmCommand(ballTransitSubsystem));
		// new JoystickButton(operatorStick, Buttons.ballIntake).whileHeld(new StartEndCommand(
		//	 () -> ballTransitSubsystem.intake(), () -> ballTransitSubsystem.intakeMotor.set(0), ballTransitSubsystem));
		// new JoystickButton(operatorStick, Buttons.ballIntake).whileHeld(new StartEndCommand(
		//	 () -> ballTransitSubsystem.intake(), () -> ballTransitSubsystem.intakeMotor.set(0), ballTransitSubsystem));
		// new JoystickButton(operatorStick, 4).whileHeld(new RunCommand(()->ballTransitSubsystem.togglePiston()));
		
		//new JoystickButton(operatorStick, Buttons.eyeballButton).whileHeld(new EyeBallCommand(driveSubsystem));

		//TESTING CODE
		//new JoystickButton(operatorStick, 3).whileHeld(()->ballTransitSubsystem.setArmAngle(PositionMode.goUp), ballTransitSubsystem);
		//new JoystickButton(operatorStick, 4).whileHeld(()->ballTransitSubsystem.setArmAngle(PositionMode.goDown), ballTransitSubsystem);
		//new JoystickButton(operatorStick, 3).whileHeld(new SequentialCommandGroup (new RunCommand(()->ballTransitSubsystem.setArmAngle(PositionMode.goDown), ballTransitSubsystem).withInterrupt(ballTransitSubsystem.lowLimitSwitch::get()), new InstantCommand(balTramsitSubsystem::togglePiston,ballTransitSubsystem)));
		// new JoystickButton(operatorStick, 4).whileHeld(new RunCommand(()->ballTransitSubsystem.togglePiston()));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return m_chooser.getSelected();
	}
}
