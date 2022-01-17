// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PIDDriveSubsystem extends SubsystemBase {
  public final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
  public final CANSparkMax leftBackMotor = new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
  public final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
  public final CANSparkMax rightBackMotor = new CANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);

  public RelativeEncoder m_leftFrontEncoder = leftFrontMotor.getEncoder();
  public RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  public RelativeEncoder m_rightFrontEncoder = rightFrontMotor.getEncoder();
  public RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();

  private SparkMaxPIDController leftFrontPIDCon = leftFrontMotor.getPIDController();
  private SparkMaxPIDController leftBackPIDCon = leftBackMotor.getPIDController();
  private SparkMaxPIDController rightFrontPIDCon = rightFrontMotor.getPIDController();
  private SparkMaxPIDController rightBackPIDCon = rightBackMotor.getPIDController();

  int smartMotionSlot = 0;
  int allowedErr;
  int minVel;
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0.000156;
  double kMaxOutput = 1;
  double kMinOutput = -1;
  double maxRPM = 5700;
  double maxVel = 2000;
  double maxAcc = 1500;
  double setPointDrive = 0;
  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);

  public PIDDriveSubsystem() {
    m_gyro.reset();
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();
    initializePID(leftFrontPIDCon);
    initializePID(leftBackPIDCon);
    initializePID(rightFrontPIDCon);
    initializePID(rightBackPIDCon);
  }

  public void manualDrive(double y, double x, double scaleX, double scaleY){
    leftFrontPIDCon.setReference(setPointLeft(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
    leftBackPIDCon.setReference(setPointLeft(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
    rightFrontPIDCon.setReference(setPointRight(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
    rightBackPIDCon.setReference(setPointRight(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
  }

  public double setPointLeft(double Jy, double Jx, double scale1, double scale2){
    setPointDrive = 1;
    double yScale = ((1 + Jy) * (1 + Math.abs(Jy) * scale2)); //abs(Jy) bc square Jy values without getting rid of the negative
    double xScale = (/*angleError(Jy, Jx)*/ scale1 * Jx);
    resetSetPoint(Jy, Jx);
    return xScale + yScale + setPointDrive;
  }

  public double setPointRight(double Jy, double Jx, double scale1, double scale2){
    setPointDrive = 1;
    double xScale = (-1 /** angleError(Jy, Jx)*/ * scale1 * Jx);
    double yScale = ((1 + Jy) * (1 + Math.abs(Jy)) * scale2);
    resetSetPoint(Jy, Jx);
    return  xScale + yScale + setPointDrive;
  }

  public void resetSetPoint(double y, double x){
    if (y == 0 & x ==0){
      setPointDrive = 0;
    }
  }

  public void autoDrive(double setPoint, double angle, double scaleLeft, double scaleRight){
    leftFrontPIDCon.setReference(autoSetPointLeft(setPoint, angle, scaleLeft), CANSparkMax.ControlType.kSmartMotion);
    leftBackPIDCon.setReference(autoSetPointLeft(setPoint, angle, scaleLeft), CANSparkMax.ControlType.kSmartMotion);
    rightFrontPIDCon.setReference(autoSetPointRight(setPoint, angle, scaleRight), CANSparkMax.ControlType.kSmartMotion);
    rightBackPIDCon.setReference(autoSetPointRight(setPoint, angle, scaleRight), CANSparkMax.ControlType.kSmartMotion);
  }

  /**
   * Sets how far encoders need to move
   * @param setPoint The displacement of the robot to the ball
   * @param angle The angle from the field (pointing torwards the rump starting at zero) to the ball
   * @param scale The number you want to muilply the angleError with to increase or decrease setPoint
   * @return if angleError is greater then zero then add to the sestpoint by angleError times scale else setpoint
   */
  public double autoSetPointLeft(double setPoint, double angle, double scale){
    if (angleError(angle) > 0){
      return (angleError(angle) * scale) + setPoint;
    }
    return setPoint;
  }
  /**
   * Sets how far encoders need to move
   * @param setPoint The displacement of the robot to the ball
   * @param angle The angle from the field (pointing torwards the rump starting at zero) to the ball
   * @param scale The number you want to muilply the angleError with to increase or decrease setPoint
   * @return if angleError is less then zero then add to the sestpoint by angleError times scale else setpoint
   */
  public double autoSetPointRight(double setPoint, double angle, double scale){
    if (angleError(angle) < 0){
      return (angleError(angle) * scale) + setPoint;
    }
    return setPoint;
  }

  public double angleError(double expectedAngle){
    double jAngle = Math.atan(1);
    /*Needs to have 2 of these bc */ 
    return Math.IEEEremainder(expectedAngle, 360) - Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public void initializePID(SparkMaxPIDController p){
    p.setP(kP);
    p.setI(kI);
    p.setD(kD);
    p.setIZone(kIz);
    p.setFF(kFF);
    p.setOutputRange(kMinOutput, kMaxOutput);
    p.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    p.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
