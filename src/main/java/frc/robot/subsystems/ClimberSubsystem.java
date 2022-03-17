// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

	int smartMotionSlot = 0;
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
	double maxVel = 2000;
	double maxAcc = 1500;
	double setPointDrive = 0;

	public DoubleSolenoid leftOuterPneumatic = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 7);
	public DoubleSolenoid leftInnerPneumatic = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 6);
	public DoubleSolenoid rightOuterPneumatic = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 5);
	public DoubleSolenoid rightInnerPneumatic = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 3, 4);

	private CANSparkMax outerMotor = new CANSparkMax(Constants.outerClimbMotorID, MotorType.kBrushless);

	public RelativeEncoder outerEncoder = outerMotor.getEncoder();

	public SparkMaxPIDController outerController = outerMotor.getPIDController();

	public double currentOuterReferencePoint = 0;
	public double currentInnerReferencePoint = 0;

	public boolean outerPIDEnabled = true;
	public boolean innerPIDEnabled = true;

	/* Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
		initializePID(outerController, outerEncoder);
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
		h.setPositionConversionFactor(1);
		h.setVelocityConversionFactor(1);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/**
	 * This will set the encoder position for the Outer PID Controller
	 * 
	 * @param position
	 */
	public void setOuterArmsPosition(double position) {
		if (outerPIDEnabled) {
			outerController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
		} else {
			outerMotor.set(0);
		}
		currentOuterReferencePoint = position;
	}

	public void disablePID() {
		setOuterPID(false);
		outerMotor.set(0);
	}

	/**
	 * This will enable/disable the Outer PID
	 * 
	 * @param val
	 */
	public void setOuterPID(boolean val) {
		outerPIDEnabled = val;
	}

	public void toggleOuterArms() { // Reverses the toggle state of the outer solenoids
		leftOuterPneumatic.toggle();
		rightOuterPneumatic.toggle();
	}

	public void toggleInnerArms() { // Reverses the toggle state of the inner solenoids
		leftInnerPneumatic.toggle();
		rightInnerPneumatic.toggle();
	}

	public double getPositionError(double expectedPosition, double currentPosition) {
		return expectedPosition - currentPosition;
	}

	public boolean outerMoveFinished() {
		return Math.abs(getPositionError(currentOuterReferencePoint, outerEncoder.getPosition())) < 1.0;
	}
}
