package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    public CANSparkMax frontLeftMotor = new CANSparkMax(Constants.leftFrontMotorDeviceID, MotorType.kBrushless);
    public CANSparkMax frontRightMotor = new CANSparkMax(Constants.rightFrontMotorDeviceID, MotorType.kBrushless);
    public CANSparkMax backLeftMotor = new CANSparkMax(Constants.leftRearMotorDeviceID, MotorType.kBrushless);
    public CANSparkMax backRightMotor = new CANSparkMax(Constants.rightRearMotorDeviceID, MotorType.kBrushless);

    public MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    public MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);
    
    DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    public DriveSubsystem() {}

    public void arcadeDrive(double moveSpeed, double rotateSpeed, double scaleConstant) {
        differentialDrive.arcadeDrive(moveSpeed * scaleConstant, rotateSpeed * scaleConstant);
    }

    @Override
    public void periodic() {

    }
}
