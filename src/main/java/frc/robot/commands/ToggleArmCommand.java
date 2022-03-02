// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallTransitSubsystem;

public class ToggleArmCommand extends CommandBase {
	private BallTransitSubsystem ballTransitSubsystem;
	
	public enum PositionMode {
		goUp,
		goDown,
		broken
	}

	private PositionMode mode = PositionMode.broken;
	//private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

	public ToggleArmCommand(BallTransitSubsystem subsystem) {
		ballTransitSubsystem = subsystem;
		addRequirements(subsystem);
	}

	@Override
	public void initialize() {
		
		if (ballTransitSubsystem.checkArmUp()) {
			ballTransitSubsystem.setMinOutput(0);
			mode = PositionMode.goDown;
			// When limit swtich on the bottom is not clicked then set mode to goDown
		} else if (ballTransitSubsystem.checkArmDown()) {
			mode = PositionMode.goUp;
		}else{
			mode = PositionMode.broken;
		}
		

		//WE WILL DELETE THIS
		/*if (ballTransitSubsystem.topLimitSwitch.get()) {
			if (ballTransitSubsystem.lowLimitSwitch.get()) {
				SmartDashboard.putString("Mulitiple Swtiches Pressed", "##ERROR##");
				// Both switches are clicked
			} else {
				mode = PositionMode.goDown;
				// When limit swtich on the bottom is not clicked then set mode to goDown
			}
		} else {
			if (ballTransitSubsystem.lowLimitSwitch.get()) {
				mode = PositionMode.goUp;
				// When limit switch on the bottom is clicked then set mode to goUp
			} else {
				SmartDashboard.putString("Limit Switches", "BROKEN!!!!");
				// not receiveing any data from switches
			}
		}*/
	}

	@Override
	public void execute() {
		if (mode == PositionMode.goDown) {
			ballTransitSubsystem.setArmAngle(mode);
		} else if (mode == PositionMode.goUp) {
			ballTransitSubsystem.setArmAngle(mode);
		}else{
			System.out.println("Were screwed");
		}
	}

	@Override
	public void end(boolean interrupted) {
		if(mode == PositionMode.goDown && interrupted){
			ballTransitSubsystem.setMinOutput(-0.3);
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (mode == PositionMode.goDown) {
			if (ballTransitSubsystem.checkArmDown()){
				ballTransitSubsystem.setMinOutput(-0.3);
				return true;
			}
			return false;
		} else if (mode == PositionMode.goUp) {
			return ballTransitSubsystem.checkArmUp();
		}
		return false;
	}

}
