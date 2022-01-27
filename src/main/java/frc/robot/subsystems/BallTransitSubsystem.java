// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class BallTransitSubsystem extends SubsystemBase {
  /*int smartMotionSlot = 0;
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
  private final CANSparkMax armMotor = new CANSparkMax(Constants.motorArmID, MotorType.kBrushless);
  private final RelativeEncoder armMotorEncoder = armMotor.getEncoder();
  private SparkMaxPIDController armPIDCon = armMotor.getPIDController();
*/
  public BallTransitSubsystem() {
   /* armPIDCon.setP(kP);
    armPIDCon.setI(kI);
    armPIDCon.setD(kD);
    armPIDCon.setIZone(kIz);
    armPIDCon.setFF(kFF);
    armPIDCon.setOutputRange(kMinOutput, kMaxOutput);
    armPIDCon.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    armPIDCon.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    armPIDCon.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    armPIDCon.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    */
  }
  public void transitUp(){
   // armPIDCon.setReference(100, CANSparkMax.ControlType.kSmartMotion);
  }
  public void transitDown(){

  }
  public void lockUpPiston(){

  }
  public void lockDownPiston(){
    
  }
  public void dropBall(){

  }
  public void intake(){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
