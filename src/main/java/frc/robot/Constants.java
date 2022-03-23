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
    public static int leftFrontMotorID = 2;//2
    public static int leftBackMotorID = 1;//1
    public static int rightFrontMotorID = 3;//3
    public static int rightBackMotorID = 4;//4
    //Intake Motor IDS
    public static int intakeArmMotorID = 8;//8
    public static int intakeMotorID = 7;//7
    //Climber Motor IDS
    public static int outerClimbMotorID = 6;

    //Limit Switches
    public static int armDownPort = 9;

    //Intake/Arm Constants
    //Negative is down and positive is up
    public static double armDownPosition = 1.2;//-10.75
    public static double armUpPosition = 12;
    public static double releaseArmPosition = 12;//5
    public static double intakeSpeed = 0.5;

    //AutoAlign Constants
    public static double neededAngle90 = 90;
    public static double neededAngleNegative90 = -90;
    public static double neededAngle0 = 0;
    public static double neededAngle180 = -180;
    public static double neededCilmbAngle = -32;

    //EyeBall Constants
    public static double yawLeftBias = 12; //Adds to Yaw
    public static double yawRightBias = -12; //Addes to Yaw
    public static double pitchOffset = 18; //Adds to Pitch
    public static double yawDriveScale = 75;
    public static double pitchDriveScale = 200;

    /* Climber Subsystem Constants */
    public static int pneumaticTimerDelay = 150;
    public static int timerDelayBetweenSteps = 48;
    public static int MLGWaterBucketClutchTime = 1; //The name is unfortunately kinda accurate

    public static double armsDownSetPoint = 1.0; //The set point for when we drive into the hangar
    public static double verticalSetPoint = 1.0; //The set point for when the arms are verticle
    public static double behindBarSetPoint = 1.0; //The set point for the inner arms when the climing starts
    public static double firstExtendSetPoint = 1.0; //The set point for when the arms move behind the bar
    public static double barAlignedSetPoint = 1.0; // When the arm is aligned to the bar when it retracts
    public static double innerArmsFinalSetPoint = 1.0; //The final place the inner arms will move to

    public static double innerArmRetractedSetPoint = 1.0;
    public static double innerArmExtendedSetPoint = 1.0;
    public static double outerArmRetractedSetPoint = 1.0;
    public static double outerArmExtendedSetPoint = 1.0;

    public static double climbFowdPosition = 18;
    public static double climbBackPosition = 0.1;

    public static final class Buttons {
        //DriverStick
        public static int inverseControl = 1;
        public static int driveBoostToggle = 0;
        public static int driveSlowToggle = 0;

        public static int turn0Toggle = 5;
        public static int turn90Toggle = 6;
        public static int turnNegative90Toggle = 7;
        public static int turn180Toggle = 8;

        public static int setDistanceButton = 4;

        //DriverStick Inverted
        public static int turn0Toggle2 = 11;
        public static int turn90Toggle2 = 12;
        public static int turnNegative90Toggle2 = 13;
        public static int turn180Toggle2 = 14;

        //TestConstants Not Used in Competition driver stick
        public static int outerArmToggle = 2;
        public static int outerClimbPosition = 15;
        public static int outerClimbVariable = 11;

        //OperatorStick
        public static int climberButton = 1;
        public static int climbBack = 16;
        public static int climbFoward = 11;
        public static int outerClimb = 4;
        public static int innerClimb = 3;

        public static int armToggle = 14;
        public static int armUp = 5;
        public static int armDown = 10;
        public static int armRelease = 15;
        public static int turnOffArm = 2;

        public static int ballIntake = 7;
        public static int ballOutTake = 8;

        public static int eyeballLeftButton = 12;
        public static int eyeballRightButton = 13;
    }

    public static final class DriveConstants {
        //Gear Ratio 8.41 to 1
        //Joystick Scale
        public static double scaleY = 0.5;
        public static double scaleX = 0.35;

        //Set speed scale
        public static double scaleFowd = 3350;
        public static double scaleTurn = 1750;

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
    }
}
