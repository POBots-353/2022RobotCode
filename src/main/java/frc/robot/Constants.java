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
    //Drive Motor IDS
    public static int leftFrontMotorID = 2;
    public static int leftBackMotorID = 1;
    public static int rightFrontMotorID = 4;
    public static int rightBackMotorID = 3;
    //Arm Motor IDS
    public static int intakeArmMotorID = 8;
    public static int intakeMotorID = 7;


    //Intake Constants
    public static double armDownPosition = 25;
    public static double armUpPosition = 0;

    //AutoAlign Constants
    public static double neededAngle90 = -90;
    public static double neededAngle45 = 45;
    public static double neededAngle180 = -180;

    /* Climber Subsystem Constants */
    public static double climbingArmLength = 1.0;
    public static double distancePerMotorClick = 1.0;
    public static double hookLengthToBase = 1.0;

    public static double armReferenceScale = 1.0;

    public static int pneumaticTimerDelay = 150;
    public static int timerDelayBetweenSteps = 1;
    public static int MLGWaterBucketClutchTime = 1; //The name is unfortunately kinda accurate

    public static double verticalSetPoint = 1.0; //The set point for when the arms are verticle
    public static double behindBarSetPoint = 1.0; //The set point for the inner arms when the climing starts
    public static double firstExtendSetPoint = 1.0; //The set point for when the arms move behind the bar
    public static double barAlignedSetPoint = 1.0; // When the arm is aligned to the bar when it retracts
    public static double innerArmsFinalSetPoint = 1.0; //The final place the inner arms will move to

    public static double innerArmRetractedSetPoint = 1.0;
    public static double innerArmExtendedSetPoint = 1.0;
    public static double outerArmRetractedSetPoint = 1.0;
    public static double outerArmExtendedSetPoint = 1.0;

    //vision constants
    public static double cameraHeight = 0;
    public static double ballHeight = 0;
    public static double cameraAngle = 0;
    public static double distanceToBall = 0; 

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
        public static int toggleArm = 1;
        public static int ballIntake = 2;
        public static int eyeballButton = 5;
    }

    public static final class DriveConstants {
        //Gear Ratio 8.41 to 1
        //Joystick Scale
        public static double scaleY = 0.5;
        public static double scaleX = 0.35;
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
        //FYI This doesn't do anything
        public static double conversionPosition = 1/*1/8.41 * Math.PI * 2 * 0.0762*/;//rotations * gear ratio * 2 * PI * wheelSize
        public static double conversionVelocity = 1 /*1/8.41 * Math.PI * 2 * 0.0762*/;//rotations * gear ratio * 2 * PI * wheelSize
        //Deacceleration Constants
        public static double deAccel = 0.98;
        public static double lowestVel = 1;
    }
}
