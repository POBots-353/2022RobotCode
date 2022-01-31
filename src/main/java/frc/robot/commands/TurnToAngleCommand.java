// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** An example command that uses an example subsystem. */
public class TurnToAngleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  
  // Use gyro declaration from above here

// The gain for a simple P loop
double kP = 1;

// Initialize motor controllers and drive
public final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
public final CANSparkMax leftBackMotor = new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
public final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
public final CANSparkMax rightBackMotor = new CANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);


MotorControllerGroup leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
MotorControllerGroup rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
// navX iniialization
AHRS gyro = new AHRS(SerialPort.Port.kUSB);


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToAngleCommand(DriveSubsystem subsystem) {
    Shuffleboard.getTab("Angle").add(gyro);
    driveSubsystem = subsystem;
    addRequirements(subsystem);
    double error = Constants.neededAngle - gyro.getAngle(); 
    // Turns the robot to face the desired direction
    drive.tankDrive(kP * error, kP * error);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
