// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ToggleArmCommand.PositionMode;


public class BallTransitSubsystem extends SubsystemBase {
  private final CANSparkMax armIntakeMotor = new CANSparkMax(Constants.intakeArmMotorID, MotorType.kBrushless);
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);

  private SparkMaxPIDController armMotorPIDCon = armIntakeMotor.getPIDController();

  private RelativeEncoder armEncoder = armIntakeMotor.getEncoder();
  private DigitalInput armDown = new DigitalInput(Constants.armDownPort);

  

 
  //private DoubleSolenoid piston = new DoubleSolenoid(0,PneumaticsModuleType.CTREPCM, 1, 1);

  int smartMotionSlot = 0;
  int allowedErr;
  int minVel;
  double kP = 4e-4;
  double kI = 0;
  double kD = 0;
  double kIz = 0;
  double kFF = 0.000156;
  double kMaxOutput = 1; 
  double kMinOutput = -.3;
  double maxVel = 3000;
  double maxAcc = 1200;

   public BallTransitSubsystem() {
     initializePID(armMotorPIDCon, armEncoder);
     resetPosition();
   }
 
  /*public void togglePiston() {
     piston.toggle();
  }
*/

  public void inTake() {
    intakeMotor.set(-Constants.intakeSpeed);
   }
 
  public void outTake() {
    intakeMotor.set(Constants.intakeSpeed);
  }

  public void turnOffIntakeMotor(){
    intakeMotor.set(0);
  }

  /**
   * Toggles piston if its out and moves the arm up
   * @param position arm up or down
   */
  public void setArmAngle(PositionMode position) {
    if (position == PositionMode.goDown) {
        armMotorPIDCon.setReference(Constants.armDownPosition, CANSparkMax.ControlType.kSmartMotion);
    } else if (position == PositionMode.goUp) {
        armMotorPIDCon.setReference(Constants.armUpPosition, CANSparkMax.ControlType.kSmartMotion);
    }else if (position == PositionMode.goUpHigher){
      armMotorPIDCon.setReference(Constants.releaseArmPosition, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  public void resetPosition(){
    armEncoder.setPosition(0);
  }

  public void releaseArm(){
    armMotorPIDCon.setReference(Constants.releaseArmPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public void turnOffArmMotor(){
    armIntakeMotor.set(0);
  }

   public boolean checkArmUp(){
     if (armEncoder.getPosition() >= Constants.armUpPosition - 0.2){
       return true;
     }
     return false;
   }

   public boolean checkArmDown(){
     if (armEncoder.getPosition() <= Constants.armDownPosition + 0.2){
       return true;
     }
     return false;
   }

  public void setMinOutput(double output){
    /*if (kMinOutput != output){
      armMotorPIDCon.setOutputRange(-0.3, output);
    }*/
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

    if(armDown.get()){
      armEncoder.setPosition(0);
    }
  }
}
