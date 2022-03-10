// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewDriveSubsystem extends SubsystemBase {

  private CANSparkMax frontLeftMotor = new CANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);

  private RelativeEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
  private RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();
  private RelativeEncoder backLeftEncoder = backLeftMotor.getEncoder();
  private RelativeEncoder backRightEncoder = backRightMotor.getEncoder();

  private SparkMaxPIDController frontLeftPIDController = frontLeftMotor.getPIDController();
  private SparkMaxPIDController frontRightPIDController = frontRightMotor.getPIDController();
  private SparkMaxPIDController backLeftPIDController = backLeftMotor.getPIDController();
  private SparkMaxPIDController backRightPIDController = backRightMotor.getPIDController();

  public final AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

  int smartMotionSlot = 0;
  int allowedErr;
  int minVel;
  double kP = 4e-4;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0.000156;
  double kMaxOutput = 1;
  double kMinOutput = -1;
  double maxVel = 4000;
  double maxAcc = 1500;

  /** Creates a new NewDriveSubsystem. */
  public NewDriveSubsystem() {
    restoreMotorDefaults();

    initializePID(frontLeftPIDController, frontLeftEncoder);
    initializePID(frontRightPIDController, frontRightEncoder);
    initializePID(backLeftPIDController, backLeftEncoder);
    initializePID(backRightPIDController, backRightEncoder);

    resetEncoders();
  }

  /**
   * Sets the speed of the motors based on the forward movement and turn movement
   * @param forward
   * @param turn
   * @param forwardScale
   * @param turnScale
   */
  public void drive(double forward, double turn, double forwardScale, double turnScale) {
    if(Math.abs(forward) > 0.05 && Math.abs(turn) > 0.1) {
      frontLeftPIDController.setReference(getLeftPoint(forward, turn, forwardScale, turnScale), ControlType.kSmartVelocity);
      frontRightPIDController.setReference(getRightPoint(forward, turn, forwardScale, turnScale), ControlType.kSmartVelocity);
      backLeftPIDController.setReference(getLeftPoint(forward, turn, forwardScale, turnScale), ControlType.kSmartVelocity);
      backRightPIDController.setReference(getRightPoint(forward, turn, forwardScale, turnScale), ControlType.kSmartVelocity);
    } else if(frontLeftEncoder.getVelocity() < 150.0 && frontRightEncoder.getVelocity() < 150.0 && turn < 0.4) {
      
    } else {
      stopDriveMotors();
    }
  }

  /**
   * Gets the RPM we should decrease the motor to
   * @param currentVelocity
   * @param expectedVelocity
   * @return The Calculated RPM decrease
   */
  private double getDecreasedRPM(double currentVelocity, double expectedVelocity) {
    double rpmError = currentVelocity - expectedVelocity;

    return (rpmError < 150.0) ? 0 : rpmError * 0.50;
  }

  /**
   * Gets the percent error of the difference between the left and right velocity
   * @param leftVelocity
   * @param rightVelocity
   * @return The Absolute Value of the Percent Error for the 2 different velocities
   */
  private double getVelocityPercentDifference(double leftVelocity, double rightVelocity) {
    return Math.abs((leftVelocity - rightVelocity) / rightVelocity) * 100;
  }

  /**
   * Stops the drive motors and disables PID
   */
  private void stopDriveMotors() {
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    backLeftMotor.set(0);
    backRightMotor.set(0);
  }

  /**
   * Gets the value for the velocity to set the left motor reference to
   * @param forward
   * @param turn
   * @param forwardScale
   * @param turnScale
   * @return The velocity for the left motors
   */
  public double getLeftPoint(double forward, double turn, double forwardScale, double turnScale) {
    double yScale = forward * forwardScale;
    double xScale = turn * turnScale;

    return yScale + xScale;
  }

  /**
   * Gets the value for the velocity to set the right motor reference to
   * @param forward
   * @param turn
   * @param forwardScale
   * @param turnScale
   * @return The velocity for the right motors
   */
  public double getRightPoint(double forward, double turn, double forwardScale, double turnScale) {
    double yScale = -forward * forwardScale;
    double xScale = turn * turnScale;

    return -(yScale + xScale);
  }

  /**
   * Gets the difference in the current angle from expected angle
   * @param expectedAngle
   * @return The Angle that is needed to turn to in the shortest rotation possible
   */
  public double getAngleError(double expectedAngle) {
    double angleSubtract = Math.IEEEremainder(expectedAngle, 360) - Math.IEEEremainder(gyro.getAngle(), 360);

    if(angleSubtract > 180) {
      return angleSubtract - 360;
    } else if(angleSubtract < -180) {
      return angleSubtract + 360;
    }
    return angleSubtract;
  }

  /**
   * Resets the Factory Defaults of all of the Drive Motors
   */
  private void restoreMotorDefaults() {
    //idk why this method actually exists but whatever
    frontLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
  }

  /**
   * Resets the position of all of the encoders to 0
   */
  private void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
  }

  /**
   * Sets the PID values for the given PID controller
   * @param PIDController
   * @param encoder
   */
  private void initializePID(SparkMaxPIDController PIDController, RelativeEncoder encoder) {
    PIDController.setP(kP);
    PIDController.setI(kI);
    PIDController.setD(kD);
    PIDController.setIZone(kIz);
    PIDController.setFF(kFF);
    PIDController.setOutputRange(kMinOutput, kMaxOutput);
    PIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    PIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    PIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    PIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
