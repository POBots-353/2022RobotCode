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
		
		/*if (checkArmUp()) {
			mode = PositionMode.goDown;
			// When limit swtich on the bottom is not clicked then set mode to goDown
		} else if (checkArmDown()) {
			mode = PositionMode.goUp;
		}else{
			mode = PositionMode.Broken;
		}
		*/

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
		//if (mode == PositionMode.goDown) {
		//	if (ballTransitSubsystem.pistonLimitSwitch.get()) {
		//		ballTransitSubsystem.togglePiston();
		//		// Takes out Piston
		//	} else {
		//		ballTransitSubsystem.setArmAngle(mode);
		//		// Moves arm down
		//	}
		//} else if (mode == PositionMode.goUp) {
		//	if (ballTransitSubsystem.pistonLimitSwitch.get()) {
		//		ballTransitSubsystem.togglePiston();
		//		// Takes out Piston
		//	} else {
		//		ballTransitSubsystem.setArmAngle(mode);
		//		// Moves arm down
		//	}
		//}
	}

	@Override
	public void end(boolean interrupted) {
		//ballTransitSubsystem.turnOffArmMotor();
	}

	/**
	 * Checks if the piston is locking.
	 * If the piston doesn't lock then this command doesn't end,
	 * so it will be using PID to hold the motors
	 * Hope this is not an issue
	 * 
	 * @return true if piston is locking, false if not
	 */
	public boolean pistonCheck() {
		//if (ballTransitSubsystem.pistonLimitSwitch.get()) {
		//	ballTransitSubsystem.turnOffMotor();
		//	return true;
		//} else {
			return false;
		//}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
	//	if (mode == PositionMode.goDown) {
	//		if (checkArmDown()) {
	//			// Checks if the lower limit is hit
	//			ballTransitSubsystem.togglePiston();
	//			return pistonCheck();
	//		} else {
	//			return false;
	//		}

	//	} else if (mode == PositionMode.goUp) {
	//		if (checkArmUp()) {
	//			// Checks if the upper limit is hit
	//			ballTransitSubsystem.togglePiston();
	//			return pistonCheck();
	//		} else {
	//			return false;
	//		}
	//	}
		return false;
	}

}
