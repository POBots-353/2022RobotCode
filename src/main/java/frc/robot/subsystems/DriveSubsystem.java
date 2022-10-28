// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);

// public final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);

  private RelativeEncoder m_leftFrontEncoder = leftFrontMotor.getEncoder();
  private RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private RelativeEncoder m_rightFrontEncoder = rightFrontMotor.getEncoder();
  private RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();

  private SparkMaxPIDController leftFrontPIDCon = leftFrontMotor.getPIDController();
  private SparkMaxPIDController leftBackPIDCon = leftBackMotor.getPIDController();
  private SparkMaxPIDController rightFrontPIDCon = rightFrontMotor.getPIDController();
  private SparkMaxPIDController rightBackPIDCon = rightBackMotor.getPIDController();

  //public final AnalogInput ultrasonic2 = new AnalogInput(1);
  int smartMotionSlot = 0;
  int allowedErr;
  int minVel;
  double kP = 4.7e-4;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0.000156;
  double kMaxOutput = 1;
  double kMinOutput = -1;
  double maxVel = 5000;
  double maxAcc = 4000;

  // The gyro sensor
  // public static final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);

  //private Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  //public static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  //private PowerDistribution powerDistributionModule = new PowerDistribution(0, ModuleType.kCTRE);
  //private PneumaticsControlModule pcm = new PneumaticsControlModule(1);

  
  public DriveSubsystem() {
    // Shuffleboard.getTab("Gyro").add(m_gyro);
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
    initializePID(leftFrontPIDCon, m_leftFrontEncoder);
    // initializePID(leftBackPIDCon, leftBackEncoder);
    initializePID(rightFrontPIDCon, m_rightFrontEncoder);
    // initializePID(rightBackPIDCon, rightBackEncoder);
    resetEncoders();
  }
  /**
   * Sets the speed of the motors
   * @param x rotation
   * @param y foward or backward
   * @param scaleX
   * @param scaleY
   */
  public void manualDrive(double x, double y, double scaleX, double scaleY) {
    //This is meant to prevent less stress on the gears of the drivetrain and accidental touch
    if (Math.abs(x) <= 0.1 && Math.abs(y) <= 0.05) {
      turnOffDriveMotors(); //Turns off PID
    } else {
      if (Math.abs(x) <= 0.09 && Math.abs(y) >= 0.5){
        x = 0;
      }
      leftFrontPIDCon.setReference(setPointLeft(x, y, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
      // leftBackPIDCon.setReference(setPointLeft(x, y, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
      rightFrontPIDCon.setReference(setPointRight(x, y, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
      // rightBackPIDCon.setReference(setPointRight(x, y, scaleX, scaleY), CANSparkMax.ControlType.kSmartVelocity);
    }
  }

  public void turnOffDriveMotors(){
    leftFrontMotor.set(0);
    rightFrontMotor.set(0);
    // leftBackMotor.set(0);
    // rightBackMotor.set(0);
  }

  /**
    Sets the velocity of the left motors in rotations per min
    If wanting to turn right, then the output of setPointLeft will
    be greater then setPointRight
    else the oppsite will occur
  */
  public double setPointLeft(double Jx, double Jy, double scaleX, double scaleY) {
    double yScale = ((Jy) * scaleY);
    double xScale = (Jx) * scaleX;
    return xScale + yScale;
  }
  
  /**
    Sets the velocity of the right motors in rotations per min
    If wanting to turn left, then the output of setPointRight will be greater then
    setPointLeft
    else the oppsite will occur
  */
  public double setPointRight(double Jx, double Jy, double scaleX, double scaleY) {
    double xScale = (-(Jx) * scaleX);
    double yScale = ((Jy) * scaleY);
    return -1 * (xScale + yScale);
  }

  public void resetEncoders() {
    m_leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
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
    //pcm.clearAllStickyFaults();
    //powerDistributionModule.clearStickyFaults();

    SmartDashboard.putNumber("Postion", leftBackEncoder.getPosition());
    SmartDashboard.putNumber("Velocity", leftBackEncoder.getVelocity());
    SmartDashboard.putNumber("Joystick x", RobotContainer.driverStick.getX());
    SmartDashboard.putNumber("Joystick y", RobotContainer.driverStick.getY());
    SmartDashboard.putNumber("Process Variable", leftBackEncoder.getVelocity());

    /*SmartDashboard.putNumber("Total Current", powerDistributionModule.getTotalCurrent());
    SmartDashboard.putNumber("Total Power", powerDistributionModule.getTotalPower());
    SmartDashboard.putNumber("Total Energy", powerDistributionModule.getTotalEnergy());
    SmartDashboard.putNumber("Voltage -_-", powerDistributionModule.getVoltage());*/

   /* SmartDashboard.putNumber("Current of Motor 4", powerDistributionModule.getCurrent(14));
    SmartDashboard.putNumber("Current of Motor 3", powerDistributionModule.getCurrent(13));
    SmartDashboard.putNumber("Current of Motor 1", powerDistributionModule.getCurrent(12));
    SmartDashboard.putNumber("Current of Motor 2", powerDistributionModule.getCurrent(15));*/
    /*SmartDashboard.putNumber("Current of Motor 0", powerDistributionModule.getCurrent(0));
    SmartDashboard.putNumber("Current of Motor 1", powerDistributionModule.getCurrent(1));
    SmartDashboard.putNumber("Current of Motor 2", powerDistributionModule.getCurrent(2));
    SmartDashboard.putNumber("Current of Motor 3", powerDistributionModule.getCurrent(3));*/
  }
}
