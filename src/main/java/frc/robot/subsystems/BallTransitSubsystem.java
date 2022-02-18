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
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ToggleArmCommand.PositionMode;


public class BallTransitSubsystem extends SubsystemBase {
   private final CANSparkMax armIntakeMotor = new CANSparkMax(Constants.intakeArmMotorID, MotorType.kBrushless);
   public final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);

  private SparkMaxPIDController armMotorPIDCon = armIntakeMotor.getPIDController();

  private RelativeEncoder armEncoder = armIntakeMotor.getEncoder();
 
  private DoubleSolenoid piston = new DoubleSolenoid(0,PneumaticsModuleType.CTREPCM, 1, 1);
  // public DigitalInput topLimitSwitch = new DigitalInput(0);
  // public DigitalInput lowLimitSwitch = new DigitalInput(0);
 
  // public DigitalInput pistonLimitSwitch = new DigitalInput(0);

  int smartMotionSlot = 0;
  int allowedErr;
  int minVel;
  double kP = 4e-4;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0.000156;
  double kMaxOutput = 1; 
  double kMinOutput = 0; //-0.25
  double maxVel = 3000;
  double maxAcc = 1200;

   public BallTransitSubsystem() {
     initializePID(armMotorPIDCon, armEncoder);
   }
// 
  // public void togglePiston() {
    // piston.toggle();
  //}

  public void setArmAngle(PositionMode position) {
    if (position == PositionMode.goDown) {
      armMotorPIDCon.setReference(1.75, CANSparkMax.ControlType.kSmartMotion);
    } else if (position == PositionMode.goUp) {
      armMotorPIDCon.setReference(12, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  public boolean pistonCheck(){
    if (piston.get() == Value.kForward){
      return true;
    }else{
      return false;
    }
  }

  public void turnOffArmMotor(){
    armIntakeMotor.set(0);
  }

  public void turnOffPiston(){
    if (piston.get() == Value.kForward){
      piston.toggle();;
    }
  }

  public void intake() {
    intakeMotor.set(0.7);
   }
 
   public void outTake() {
    intakeMotor.set(-0.7);
   }

   public boolean checkArmUp(){
     if (armEncoder.getPosition() >= 11.5){
       return true;
     }
     return false;
   }

   public boolean checkArmDown(){
     if (armEncoder.getPosition() <= 1.75){
       return true;
     }
     return false;
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
    SmartDashboard.putNumber("Position of Arm", armEncoder.getPosition());
  }
}
