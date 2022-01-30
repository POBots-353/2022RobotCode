// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.imageio.ImageTypeSpecifier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);

  private RelativeEncoder m_leftFrontEncoder = leftFrontMotor.getEncoder();
  private RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private RelativeEncoder m_rightFrontEncoder = rightFrontMotor.getEncoder();
  private RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();

  private SparkMaxPIDController leftFrontPIDCon = leftFrontMotor.getPIDController();
  private SparkMaxPIDController leftBackPIDCon = leftBackMotor.getPIDController();
  private SparkMaxPIDController rightFrontPIDCon = rightFrontMotor.getPIDController();
  private SparkMaxPIDController rightBackPIDCon = rightBackMotor.getPIDController();

  public final AnalogInput input = new AnalogInput(0);
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
  double maxRPM = 5700;
  double maxVel = 4000;
  double maxAcc = 1500;
  double setPointDrive = 0;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);

  public DriveSubsystem() {
    m_gyro.reset();
    Shuffleboard.getTab("Example tab").add(m_gyro);
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();
    initializePID(leftFrontPIDCon, m_leftFrontEncoder);
    initializePID(leftBackPIDCon, leftBackEncoder);
    initializePID(rightFrontPIDCon, m_rightFrontEncoder);
    initializePID(rightBackPIDCon, rightBackEncoder);
    leftBackEncoder.setPosition(0);

  }

  public void manualDrive(double y, double x, double scaleX, double scaleY) {
    leftFrontPIDCon.setReference(setPointLeft(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
    leftBackPIDCon.setReference(setPointLeft(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
    rightFrontPIDCon.setReference(setPointRight(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
    rightBackPIDCon.setReference(setPointRight(y, x, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
  }

  public double setPointLeft(double Jy, double Jx, double scale1, double scale2) {
    double yScale = ((Jy) * scale2);
    double xScale = (Jx) * scale1;
    /*
     * if (Jy < .1){
     * xScale = -xScale;
     * }
     */
    return xScale + yScale;
  }

  public double setPointRight(double Jy, double Jx, double scale1, double scale2) {
    double xScale = (-(Jx) * scale1);
    double yScale = ((Jy) * scale2);
    /*
     * if (Jy < .1){
     * xScale = -xScale;
     * }
     */
    return -1 * (xScale + yScale);
  }

  public void autoDrive(double displacement) {
    leftFrontPIDCon.setReference(displacement,
        CANSparkMax.ControlType.kSmartMotion);
    leftBackPIDCon.setReference(displacement, CANSparkMax.ControlType.kSmartMotion);
    rightFrontPIDCon.setReference(displacement,
        CANSparkMax.ControlType.kSmartMotion);
    rightBackPIDCon.setReference(displacement,
        CANSparkMax.ControlType.kSmartMotion);
  }

  public boolean pointReached(double displacement) {
    if (displacement < m_leftFrontEncoder.getPosition()) {
      resetEncoders();
      return true;
    }
    return false;
  }

  public void resetEncoders() {
    m_leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }

  public double getDistance() {
    return m_leftFrontEncoder.getPosition();
  }

  // Neeed to get rid of this soon
  public double angleError(double expectedAngle) {
    return Math.IEEEremainder(expectedAngle, 360) - Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public void initializePID(SparkMaxPIDController p, RelativeEncoder h) {
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
    double processVariable = leftBackEncoder.getVelocity();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("UltraSonic", (input.getValue() * 0.034) / 2 * 2.54);
    SmartDashboard.putNumber("Postion", leftBackEncoder.getPosition());
    SmartDashboard.putNumber("Velocity", leftBackEncoder.getVelocity());
    SmartDashboard.putNumber("Joystick x", RobotContainer.driverStick.getX());
    SmartDashboard.putNumber("Joystick y", RobotContainer.driverStick.getY());
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", leftBackMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
