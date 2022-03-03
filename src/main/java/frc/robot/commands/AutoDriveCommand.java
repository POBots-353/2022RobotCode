// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {
  /**
   * This class is not meant to add any auto code
   * it was built so that it can end after a condition is reached
   * and make it easier to code autoNav
   */
  public static boolean collisionDetected = false;
  double lastLinearAccelerationY;
  final static double kCollisionThreshold_DeltaG = 0.5f;
  
  private final DriveSubsystem driveSubsystem;
  private double displacement;

  public AutoDriveCommand(DriveSubsystem drive, double displacement) {
    driveSubsystem = drive;
    this.displacement = displacement;
    addRequirements(driveSubsystem);
  }

  // Resets Encoders once the method runs to prevent the robot from doing other
  // movements
  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  //Moves Robot to set position
  @Override
  public void execute() {
    //Moves Robo to specfic Distance
    //(Rotations/8.41)*2*pi* (Radius of the wheel) (Inches or meters) = Displacement
    //(Displacement/ (2*pi* radius of the wheel)) * 8.41 = Rotations
    //ONLY INPUT ROTATIONS 
    driveSubsystem.autoDrive(displacement);//Add to the displacement in speed up the motors
    //To detect collision
    double currLinearAccelerationY = DriveSubsystem.m_gyro.getWorldLinearAccelY();
    double jerkY = currLinearAccelerationY - lastLinearAccelerationY;
    lastLinearAccelerationY = currLinearAccelerationY;
    
    //We don't need this
    //Its just there as an option
    if(Math.abs(jerkY) > kCollisionThreshold_DeltaG){
      collisionDetected = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      driveSubsystem.resetEncoders();
    }
    //This will stop the motors from approaching the added one and stop at the current displacement
    driveSubsystem.turnOffDriveMotors();
  }

  // Returns true when the point is reached and resets the encoders
  @Override
  public boolean isFinished() {
    return driveSubsystem.pointReached(displacement);
  }
}
