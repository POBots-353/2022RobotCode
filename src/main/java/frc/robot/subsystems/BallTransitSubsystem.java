// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.imageio.ImageTypeSpecifier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class BallTransitSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(6, MotorType.kBrushless);
    //private final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeArmMotorID, MotorType.kBrushless);
    private final CANSparkMax shooterMotor = new CANSparkMax(8, MotorType.kBrushless);
    public static final CANSparkMax armIntakeMotor = new CANSparkMax(Constants.intakeArmMotorID, MotorType.kBrushless);
    public static final RelativeEncoder armEncoder = armIntakeMotor.getEncoder();
    // private SparkMaxPIDController intakeMotorPIDCon =
    //intakeMotor.getPIDController();
    // public RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    // public DoubleSolenoid upperPiston = new
    // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 1);
    // public DoubleSolenoid lowerPiston = new
    // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 1);
    // DigitalInput toplimitSwitch = new DigitalInput(0);
    // DigitalInput lowlimitSwitch = new DigitalInput(0);

    // int smartMotionSlot = 0;
    // int allowedErr;
    // int minVel;
    // double kP = 4e-4;
    // double kI = 0;
    // double kD = 0;
    // double kIz = 0;
    // double kFF = 0.000156;
    // double kMaxOutput = 1;
    // double kMinOutput = -1;
    // double maxRPM = 5700;
    // double maxVel = 4000;
    // double maxAcc = 1500;
    // double setPointDrive = 0;

    double goal = 1000;
    //double encoderError = goal - intakeEncoder.getPosition();
    double kP = .4;
    double neededAngle = 0;
    public boolean downPistonPosition = false;
    public boolean upPistonPosition = true;
    /** Creates a new BallTransitSubsystem. */
    public BallTransitSubsystem() {
        // initializePID(intakeMotorPIDCon, intakeEncoder);
    }
    public void turnOffArm(){
        armIntakeMotor.set(0);
    }
    public void toggleShooter(boolean y) {
        if (y) {
            shooterMotor.set(-0.7);
        } else {
            shooterMotor.set(0);
        }
    }

    public void toggleIntake(boolean y) {
        if (y) {
            intakeMotor.set(0.7);
        } else {
            intakeMotor.set(0);
        }

    }

    public void toggleDownLock() {
        if (downPistonPosition) {
            downPistonPosition = false;
        } else if (downPistonPosition != true) {
            downPistonPosition = true;
        }
    }

    public void toggleUpLock() {
        if (upPistonPosition) {
            upPistonPosition = false;
        } else if (upPistonPosition != true) {
            upPistonPosition = true;
        }
    }

    public boolean getDownPiston() {
        return downPistonPosition;
    }

    public boolean getUpPiston() {
        return upPistonPosition;
    }

    /*
     * public void toggleUpPiston() {
     * if (toplimitSwitch.get()) {
     * upperPiston.set(Value.kForward);
     * }else{
     * upperPiston.set(Value.kReverse);
     * }
     * // if top limit switch is hit, extends piston to lock
     * }
     * 
     * public void toggleDownPiston() {
     * if (lowlimitSwitch.get()) {
     * lowerPiston.set(Value.kForward);
     * // // if bottom limit switch is hit, extends piston to lock
     * }else{
     * lowerPiston.set(Value.kReverse);
     * }
     * }
     * public boolean getDownPiston(){
     * return lowerPiston.get();
     * }
     * public boolean getUpPiston(){
     * return upperPiston.get();
     * }
     * public void transitUp() {
     * intakeMotorPIDCon.setReference(1.0, CANSparkMax.ControlType.kSmartMotion);
     * }
     * 
     * public void transitDown() {
     * intakeMotorPIDCon.setReference(-1.0, CANSparkMax.ControlType.kSmartMotion);
     * }
     * 
     * public void dropBall() {
     * 
     * }
     * 
     * public void intake() {
     * 
     * }
     */
    // public void initializePID(SparkMaxPIDController p, RelativeEncoder h){
    // p.setP(kP);
    // p.setI(kI);
    // p.setD(kD);
    // p.setIZone(kIz);
    // p.setFF(kFF);
    // p.setOutputRange(kMinOutput, kMaxOutput);
    // p.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    // p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    // p.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    // p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    // h.setPositionConversionFactor(DriveConstants.conversionPosition);
    // h.setVelocityConversionFactor(DriveConstants.conversionVelocity);
    // }
     @Override
     public void periodic() {
        
    // // This method will be called once per scheduler run
     }
}
