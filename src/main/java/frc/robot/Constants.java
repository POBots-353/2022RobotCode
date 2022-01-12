// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Joystick and Controller Ports
    public static int driverStickPort = 0;

    //CANSparkMax Device ID constants
	public static int leftFrontMotorDeviceID = 4; 
	public static int leftRearMotorDeviceID = 2;
	public static int rightFrontMotorDeviceID = 1;
	public static int rightRearMotorDeviceID = 3;

	//Driver button constants
	public static int turboButtonNumber = 2;
	public static int slowButtonNumber = 3;

	//Constants for driving modes within arcadeDrive()
	public static double turboScale = 1.00; //faster speed for the robot's drivetrain
	public static double slowScale = 0.5; //Slower speed for the robot's drivetrain
	public static double driverScale = 0.88; //Original speed for the robot's drivetrain
}
