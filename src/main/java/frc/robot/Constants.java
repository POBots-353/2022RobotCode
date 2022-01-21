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
    public static int leftFrontMotorID = 4;
    public static int leftBackMotorID = 2;
    public static int rightFrontMotorID = 1;
    public static int rightBackMotorID = 3;

    /*Climber Subsystem Constants*/
    public static double climbingArmLength = 1.0;
    public static double distancePerMotorClick = 1.0;
    public static double hookLengthToBase = 1.0;

    public static double verticleSetPoint = 1.0;
    public static double firstExtendPoint = 1.0;
    public static double barAlignedExtendPoint = 1.0; // When the arm is aligned to the bar when it retracts

    public static double scaleY = 4000;
    public static double scaleX = 4000;
    public static double conversionPosition = 1;
    public static double conversionVelocity = 1;
}
