// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

 package frc.robot.commands;

  import edu.wpi.first.wpilibj2.command.CommandBase;
  import frc.robot.subsystems.BallTransitSubsystem;

 public class IntakeBallCommand extends CommandBase {
   private final BallTransitSubsystem ballTransitSubsystem;
   public IntakeBallCommand(BallTransitSubsystem subsystem) {
     ballTransitSubsystem = subsystem;
     addRequirements(ballTransitSubsystem);
   }

//   // Called when the command is initially scheduled.
   @Override
   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
   public void execute() {

    if (ballTransitSubsystem.getUpPiston()){
      ballTransitSubsystem.toggleUpLock();
     }
     if(!ballTransitSubsystem.getDownPiston()){
       //ballTransitSubsystem.transitDown();//Piston should be locked after this method is complete
        ballTransitSubsystem.downPistonPosition = true;
     }else{
       ballTransitSubsystem.toggleIntake(true);
     }
   }

//   //Turns off the motor after the command ends
   @Override
   public void end(boolean interrupted) {
     if(interrupted){
       ballTransitSubsystem.toggleIntake(false);
     }
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
    return false;
   }
 }
