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

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);

  public PIDDriveSubsystem() {
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();
    initializePID(leftFrontPIDCon);
    initializePID(leftBackPIDCon);
    initializePID(rightFrontPIDCon);
    initializePID(rightBackPIDCon);
  }
  public void manualDrive(double y, double x, double setPoint){
    double scale1 = 0, scale2 = 0;
    leftFrontPIDCon.setReference(setPointLeft(y, x, scale1, scale2, setPoint), CANSparkMax.ControlType.kSmartVelocity);
    leftBackPIDCon.setReference(setPointLeft(y, x, scale1, scale2, setPoint), CANSparkMax.ControlType.kSmartVelocity);
    rightFrontPIDCon.setReference(setPointRight(y, x, scale1, scale2, setPoint), CANSparkMax.ControlType.kSmartVelocity);
    rightBackPIDCon.setReference(setPointRight(y, x, scale1, scale2, setPoint), CANSparkMax.ControlType.kSmartVelocity);
  }
  public double setPointLeft(double Jy, double Jx, double scale1, double scale2, double setPoint){
    double yScale = ((1 + Jy) * (1 + Math.abs(Jy) * scale2)); //abs(Jy) bc square Jy values without getting rid of the negative
    double xScale = (angleError(Jy, Jx) * scale1 * Jx);
    return xScale + yScale + setPoint;
  }
  public double setPointRight(double Jy, double Jx, double scale1, double scale2, double setPoint){
    double xScale = (-1 * angleError(Jy, Jx) * scale1 * Jx);
    double yScale = ((1 + Jy) * (1 + Math.abs(Jy)) * scale2);
    return xScale + yScale + setPoint;
  }
  /**
   * Can add PID to this if the curve seems wonky
   * Doesn't need to be negated bc the Jx will be muiltplied by angleError
   * @param Jy Joystick y value
   * @param Jx Joystick x value
   * @return The amount of degrees the robot needs to turn
  */
  public double angleError(double Jy, double Jx){
    double jAngle = Math.atan(Jy/Jx);
    if (Jy > 0 & Jy != 0){
      return Math.abs(Math.IEEEremainder(m_gyro.getAngle(), 360) - Math.IEEEremainder(jAngle, 360));
    }else if (Jy < 0 & Jy != 0){
      return Math.abs(Math.IEEEremainder(m_gyro.getAngle() + 180, 360) - Math.IEEEremainder(jAngle, 360));
    }else{
      return 0.0;
    }
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
