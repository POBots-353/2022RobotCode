// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Motor IDs
    public static int leftFrontMotorID = 4;
    public static int leftBackMotorID = 2;
    public static int rightFrontMotorID = 1;
    public static int rightBackMotorID = 3;
    public static int motorArmID = 0;

    //AutoAlign Constants
    public static double neededAngle90 = -90;
    public static double neededAngle45 = 45;
    public static double neededAngle180 = -180;

    /* Climber Subsystem Constants */
    public static double climbingArmLength = 1.0;
    public static double distancePerMotorClick = 1.0;
    public static double hookLengthToBase = 1.0;

    public static double verticleSetPoint = 1.0;
    public static double firstExtendPoint = 1.0;
    public static double barAlignedExtendPoint = 1.0; // When the arm is aligned to the bar when it retracts

    public static final class Buttons {
        //DriverStick
        public static int inverseControl = 1;
        public static int driveBoostToggle = 0;
        public static int driveSlowToggle = 0;
        public static int turn45Toggle = 2;
        public static int turn90Toggle = 3;
        public static int turn180Toggle = 4;
        public static int turnWithJoyStick = 0;
        //OperatorStick
        public static int climberButton = 0;
        public static int stopClimb = 0;
        public static int dumpBallToggle = 0;
    }

    public static final class DriveConstants {
        //Gear Ratio 8.41 to 1
        //Joystick Scale
        public static double scaleY = 0.5;
        public static double scaleX = 0.4;
        //Set speed scale
        public static double scaleFowd = 4000;
        public static double scaleTurn = 1000;
        //Slow Drive
        public static double scaleYSlow = 4000;
        public static double scaleTurnSlow = 4000;
        //Fast Drive
        public static double scaleYBoost = 4000;
        public static double scaleTurnBoost = 4000;
        //Encoder Converstion
        public static double conversionPosition = 1/*1/8.41 * Math.PI * 2 * 0.0762*/;//rotations * gear ratio * 2 * PI * wheelSize
        public static double conversionVelocity = 1 /*1/8.41 * Math.PI * 2 * 0.0762*/;//rotations * gear ratio * 2 * PI * wheelSize
        //Deacceleration Constants
        public static double deAccel = 0.98;
        public static double lowestVel = 1;
    }
}
