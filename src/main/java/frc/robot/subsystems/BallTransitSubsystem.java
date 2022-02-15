// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ToggleArmCommand.PositionMode;


public class BallTransitSubsystem extends SubsystemBase {
   private final CANSparkMax armIntakeMotor = new CANSparkMax(8, MotorType.kBrushless);
  // public final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);

  private SparkMaxPIDController armMotorPIDCon = armIntakeMotor.getPIDController();

  private RelativeEncoder armEncoder = armIntakeMotor.getEncoder();
// 
  // private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 1);
// 
  // public DigitalInput topLimitSwitch = new DigitalInput(0);
  // public DigitalInput lowLimitSwitch = new DigitalInput(0);
// 
  // public DigitalInput lowPistonLimitSwitch = new DigitalInput(0);
  // public DigitalInput topPistonLimitSwitch = new DigitalInput(0);

  int smartMotionSlot = 0;
  int allowedErr;
  int minVel;
  double kP = 4e-4;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0.000156;
  double kMaxOutput = 1; // adjust the values to not fully drop the intake
  double kMinOutput = 0; // adjust the values min around 0.2
  double maxVel = 4000;
  double maxAcc = 1500;
  double setPointDrive = 0;

   public BallTransitSubsystem() {
     initializePID(armMotorPIDCon, armEncoder);
   }
// 
  // public void togglePiston() {
    // piston.toggle();
  //}

  public void setArmAngle(PositionMode position) {
    if (position == PositionMode.goDown) {
      armMotorPIDCon.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    } else if (position == PositionMode.goUp) {
      armMotorPIDCon.setReference(15, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  public void turnOffArmMotor(){
    armIntakeMotor.set(0);
  }
  //public void intake() {
    // if (topLimitSwitch.get()) {
      // intakeMotor.set(-0.4);
    // } else if (lowLimitSwitch.get()) {
      // intakeMotor.set(0.4);
    // } else {
      // intakeMotor.set(0);
    // }
   //}
 
  // public void inverseIntake() {
    // if (topLimitSwitch.get()) {
      // intakeMotor.set(0.4);
    // } else if (lowLimitSwitch.get()) {
      // intakeMotor.set(-0.4);
    // } else {
      // intakeMotor.set(0);
    // }
  // }
// 
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
    //This method will be called once per scheduler run
  }
 }
