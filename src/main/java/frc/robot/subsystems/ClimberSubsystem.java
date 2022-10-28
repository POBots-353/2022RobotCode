// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	public DoubleSolenoid leftOuterPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 7);
	public DoubleSolenoid leftInnerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 6);
	public DoubleSolenoid rightOuterPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
	public DoubleSolenoid rightInnerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 5);

	/* Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// SmartDashboard.putBoolean("Compressor On", pcmCompressor.enabled());
	}

	public void toggleOuterArms() { // Reverses the toggle state of the outer solenoids
		if(leftOuterPneumatic.get() == Value.kOff){
			leftOuterPneumatic.set(Value.kReverse);
			rightOuterPneumatic.set(Value.kReverse);
		}
		leftOuterPneumatic.toggle();
		rightOuterPneumatic.toggle();
	}

	 public void toggleInnerArms() { // Reverses the toggle state of the inner solenoids
		if (leftInnerPneumatic.get() == Value.kOff) {
			leftInnerPneumatic.set(Value.kReverse);
			rightInnerPneumatic.set(Value.kReverse);
		}
		 leftInnerPneumatic.toggle();
	 	 rightInnerPneumatic.toggle();
	 }

	
  	public void enableCompressor() {
  	//   pcmCompressor.enableDigital();
 	 }

  	public void disableCompressor(){
 	//   pcmCompressor.disable(); 
 	 }
}
