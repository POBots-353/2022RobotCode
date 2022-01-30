// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
public class BallTransitSubsystem extends SubsystemBase {
  /*private final Solenoid armLockUp = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final Solenoid armLockDown = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  */
  public BallTransitSubsystem() {
  }
  public void transitUp(){
  }
  public void transitDown(){
  }
  public void toggleUpLock(){
    //armLockUp.toggle();
  }
  public void toggleDownLock(){
    //armLockDown.toggle();
  }
  public boolean getlockUpPiston(){
    return false;//armLockUp.get();
  }
  public boolean getlockDownPiston(){
    return false; //armLockDown.get();
  }
  /**
   * Releases the ball
   * @param yes True to run motor and false to stop the motor
   */
  public void dropBall(boolean yes){
    
  }
  public void intake(boolean yes){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
