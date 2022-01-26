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

    /* Climber Subsystem Constants */
    public static double climbingArmLength = 1.0;
    public static double distancePerMotorClick = 1.0;
    public static double hookLengthToBase = 1.0;

    public static int pneumaticTimerDelay = 150;
    public static int timerDelayBetweenSteps = 1;

    public static double verticleSetPoint = 1.0; //The set point for when the arms are verticle
    public static double innerArmsOriginalSetPoint = 1.0; //The set point for the inner arms when the climing starts
    public static double firstExtendSetPoint = 1.0; //The set point for when the arms move behind the bar
    public static double barAlignedSetPoint = 1.0; // When the arm is aligned to the bar when it retracts

    public static double innerArmRetractedSetPoint = 1.0;
    public static double innerArmExtendedSetPoint = 1.0;
    public static double outerArmRetractedSetPoint = 1.0;
    public static double outerArmExtendedSetPoint = 1.0;

    public static final class Buttons {
        //DriverStick
        public static int inverseControl = 0;
        public static int driveBoostToggle = 0;
        public static int driveSlowToggle = 0;
        public static int turn45Toggle = 0;
        public static int turn90Toggle = 0;
        public static int turn180Toggle = 0;
        public static int turnWithJoyStick = 0;
        //OperatorStick
        public static int climberButton = 0;
        public static int stopClimb = 0;
        public static int dumpBallToggle = 0;
    }

    public static final class DriveConstants {
        //Standard Drive
        public static double scaleY = 4000;
        public static double scaleTurn = 4000;
        //Slow Drive
        public static double scaleYSlow = 4000;
        public static double scaleTurnSlow = 4000;
        //Fast Drive
        public static double scaleYBoost = 4000;
        public static double scaleTurnBoost = 4000;

        public static double conversionPosition = 1;
        public static double conversionVelocity = 1;
    }
}
