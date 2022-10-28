// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.DriveConstants;
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

	public static final Joystick driverStick = new Joystick(0);
	public static final Joystick operatorStick = new Joystick(1);

	//Compressor
	

	public RobotContainer() {
		
		// Default Drive
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
	 * Also, be careful about brownouts (if to many things are running at once it-
	 * will cause it), especailly if your running PID on motors and neumatics, so
	 * make sure it
	 * gets disabled after use execept drive
	 */
	private void configureButtonBindings() {
		// Climber
		new JoystickButton(driverStick, Buttons.innerClimb).whenPressed(climberSubsystem::toggleInnerArms, climberSubsystem);
		new JoystickButton(driverStick, Buttons.outerClimb).whenPressed(climberSubsystem::toggleOuterArms, climberSubsystem);

		new JoystickButton(operatorStick, 6).whenPressed(climberSubsystem::enableCompressor, climberSubsystem);
		new JoystickButton(operatorStick, 9).whenPressed(climberSubsystem::disableCompressor, climberSubsystem);
	}
	
	// Runs the auto command
	public Command getAutonomousCommand() {
		return new RunCommand(() -> {});
	}
}
