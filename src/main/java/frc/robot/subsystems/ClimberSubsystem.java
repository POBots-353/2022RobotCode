// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
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
	double kMaxOutput = 0.15;
	double kMinOutput = -0.15;
	double maxRPM = 5700;
	double maxVel = 2000;
	double maxAcc = 1500;
	double setPointDrive = 0;

	//private Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
	public DoubleSolenoid leftOuterPneumatic = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 7); //all were mapped to 1 before
	public DoubleSolenoid leftInnerPneumatic = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 1, 6); //changong pcm means it defaults to 0
	public DoubleSolenoid rightOuterPneumatic = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 3, 4);
	public DoubleSolenoid rightInnerPneumatic = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 2, 5);

	private CANSparkMax outerMotor = new CANSparkMax(Constants.outerClimbMotorID, MotorType.kBrushless);

	public RelativeEncoder outerEncoder = outerMotor.getEncoder();

	public SparkMaxPIDController outerController = outerMotor.getPIDController();

	public double currentOuterReferencePoint = 0;
	public double currentInnerReferencePoint = 0;

	public boolean outerPIDEnabled = true;
	public boolean innerPIDEnabled = true;

	/* Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
		SmartDashboard.putNumber("Climber Position", 0);
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
		SmartDashboard.putNumber("Climber Current", outerEncoder.getPosition());
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
			disablePID();
		}
		currentOuterReferencePoint = position;
	}

	public void disablePID() {
		//setOuterPID(false);
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

	public void climberStop(){
		outerController.setReference(0 ,CANSparkMax.ControlType.kSmartMotion);
	}
	public void climberOff(){
		outerMotor.set(0);
	}
	public void toggleOuterArms() { // Reverses the toggle state of the outer solenoids
		if(leftOuterPneumatic.get() == Value.kOff){
			leftOuterPneumatic.set(Value.kReverse);
			rightOuterPneumatic.set(Value.kReverse);
		}
		leftOuterPneumatic.toggle();
		rightOuterPneumatic.toggle();
		if (leftOuterPneumatic.get() == Value.kReverse || rightOuterPneumatic.get() == Value.kReverse){
			outerMotor.set(0);
		}
	}

	 public void toggleInnerArms() { // Reverses the toggle state of the inner solenoids
		if(leftInnerPneumatic.get() == Value.kOff){
			leftInnerPneumatic.set(Value.kReverse);
			rightInnerPneumatic.set(Value.kReverse);
		}
		 leftInnerPneumatic.toggle();
	 	 rightInnerPneumatic.toggle();
		  //leftInnerPneumatic.set(Value.kReverse);
			//rightInnerPneumatic.set(Value.kReverse);
		 //leftInnerPneumatic.set(Value.kReverse);
			//rightInnerPneumatic.set(Value.kReverse);
		//SmartDashboard.putBoolean("Inner Arm", Value.kForward == );
	 }

	public double getPositionError(double expectedPosition, double currentPosition) {
		return expectedPosition - currentPosition;
	}

	public boolean outerMoveFinished() {
		return Math.abs(getPositionError(currentOuterReferencePoint, outerEncoder.getPosition())) < 1.0;
	}

	
  	public void enableCompressor(){
  	  //pcmCompressor.enableDigital();
 	 }

  	public void disableCompressor(){
 	  // pcmCompressor.disable(); 
 	 }

	public void moveForeward(){
		outerMotor.set(-.08);
		//outerController.setReference(-3.6, CANSparkMax.ControlType.kSmartMotion);
	}
	public void moveBackward(){
		outerMotor.set(.08);
		//outerController.setReference(-Constants.climbBackPosition, CANSparkMax.ControlType.kSmartMotion);
	}

	public void moveClimberArms(double position){
		//position = SmartDashboard.getNumber("Climber Position", 2);
		//outerMotor.set(.10);
		outerController.setReference(-position, CANSparkMax.ControlType.kSmartMotion);
	}
}
