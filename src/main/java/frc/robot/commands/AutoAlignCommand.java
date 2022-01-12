// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlignCommand extends CommandBase {

  
public I2C.Port port = I2C.Port.kOnboard;

public ColorSensorV3 colorSensor = new ColorSensorV3(port); 

public Color detectedColor = new Color(255, 255, 255);
public final Color black = new Color(0, 0, 0);

public DriveSubsystem driveSystem;

  /** Creates a new AutoAlignCommand. */
  public AutoAlignCommand(DriveSubsystem system) {
    driveSystem = system;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    detectedColor = colorSensor.getColor();
    checkColor(detectedColor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void checkColor(Color color) {
    if(color.equals(black)) {
      driveSystem.arcadeDrive(0, 0, 0);
    }
  }

}
