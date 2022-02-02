// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.ClimberSubsystem;

// public class ClimberCommand extends CommandBase {

//   int climbingStage = 0;

//   int timer = 0;
//   int timerStarted = 0;
//   int timerLength = 0;

//   private final ClimberSubsystem climberSubsystem;

//   /* Creates a new ClimberCommand. */
//   public ClimberCommand(ClimberSubsystem climber) {
//     climberSubsystem = climber;
//     addRequirements(climberSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double leftOuterPosition = climberSubsystem.m_leftOuterEncoder.getPosition();
//     double leftInnerPosition = climberSubsystem.m_leftInnerEncoder.getPosition();
//     double rightOuterPosition = climberSubsystem.m_rightOuterEncoder.getPosition();
//     double rightInnerPosition = climberSubsystem.m_rightInnerEncoder.getPosition();
    
//     climberSubsystem.setOuterArmsPosition(climberSubsystem.currentOuterReferencePoint);
//     climberSubsystem.setInnerArmsPosition(climberSubsystem.currentInnerReferencePoint);

//     switch (climbingStage) {
//       case 0:
//         /*
//          * At the end of this stage the outer arms are extended to reach the second bar
//          */
//         climberSubsystem.oldToggleOuterArms();
//         startTimer(Constants.timerDelayBetweenSteps);
//         climbingStage = 1;
//         break;
//       case 1:
//         /*
//          * At the end of this stage the robot is moved forward to align with the bar
//          */
//         if (timerCompleted()) {
//           resetTimer();
//           // TODO - harass jason to add a drive thing
//           startTimer(Constants.timerDelayBetweenSteps);
//           climbingStage = 2;
//         }
//         break;
//       case 2:
//         /*
//          * At the end of this stage the outer arms are retracted and the robot is
//          * hanging on the 2nd bar
//          */
//         if (timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleOuterArms();
//           startTimer(Constants.timerDelayBetweenSteps);
//           climbingStage = 3;
//         }
//         break;
//       case 3:
//         /*
//          * At the end of this stage we start moving the inner arms towards the 3rd bar
//          */
//         if (timerCompleted()) {
//           resetTimer();
//           climberSubsystem.setInnerArmsPosition(Constants.firstExtendSetPoint);
//           startTimer(Constants.timerDelayBetweenSteps);
//           climbingStage = 4;
//         }
//         break;
//       case 4:
//         /*
//          * At the end of this stage we start moving the inner arms to be aligned with
//          * the bar
//          */
//         if (leftInnerPosition == Constants.firstExtendSetPoint
//             && timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleInnerArms();
//           climberSubsystem.setInnerArmsPosition(Constants.barAlignedSetPoint);
//           startTimer(Constants.timerDelayBetweenSteps);
//           climbingStage = 5;
//         }
//         break;
//       case 5:
//         /*
//          * At the end of this stage the Inner arms are on bar 3 and we start moving the
//          * outer ones towards bar 4
//          */
//         if (leftInnerPosition == Constants.barAlignedSetPoint
//             && timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleInnerArms();
//           /* There may be a delay here */
//           startTimer(Constants.pneumaticTimerDelay);
//           climbingStage = 6;
//         }
//         break;
//       case 6:
//         if (timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleOuterArms();
//           climberSubsystem.setOuterArmsPosition(Constants.firstExtendSetPoint);
//           startTimer(Constants.timerDelayBetweenSteps);
//           climbingStage = 7;
//         }
//         break;
//       case 7:
//         /*
//          * At the end of this stage we start moving the outer arms towards the
//          * transversal bar
//          */
//         if (leftOuterPosition == Constants.firstExtendSetPoint
//             && climberSubsystem.leftOuterArmPneumatic.get() == Value.kReverse
//             && timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleOuterArms();
//           climberSubsystem.setOuterArmsPosition(Constants.barAlignedSetPoint);
//           startTimer(Constants.timerDelayBetweenSteps);
//           climbingStage = 8;
//         } else if (climberSubsystem.leftOuterArmPneumatic.get() == Value.kForward) {
//           climberSubsystem.oldToggleOuterArms();
//         }
//         break;
//       case 8:
//         /*
//          * At the end of this stage we get the outer arms on the transversal bar and
//          * release the inner
//          */
//         if (leftOuterPosition == Constants.barAlignedSetPoint
//             && timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleOuterArms();
//           /* There may be a delay here */
//           startTimer(Constants.pneumaticTimerDelay);
//           climbingStage = 9;
//         }
//         break;
//       case 9:
//         if (timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleInnerArms();
//           startTimer(Constants.timerDelayBetweenSteps);
//           climbingStage = 10;
//         }
//         break;
//       case 10:
//         if (leftInnerPosition == Constants.firstExtendSetPoint
//             && climberSubsystem.leftInnerArmPneumatic.get() == Value.kReverse
//             && timerCompleted()) {
//           resetTimer();
//           climberSubsystem.oldToggleInnerArms();
//           climberSubsystem.setInnerArmsPosition(Constants.firstExtendSetPoint);
//           climbingStage = 16;
//         } else if (climberSubsystem.leftInnerArmPneumatic.get() == Value.kForward) {
//           climberSubsystem.oldToggleInnerArms();
//         }
//       default:
//         /* If this function gets called then we're screwed */
//         break;
//     }
//   }

//   // Called once{} the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     climberSubsystem.leftOuterMotor.set(0);
//     climberSubsystem.leftInnerMotor.set(0);
//     climberSubsystem.rightOuterMotor.set(0);
//     climberSubsystem.rightInnerMotor.set(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   public void updateTimer() {
//     timer++;
//   }

//   public void startTimer(int length) {
//     timerStarted = timer;
//     timerLength = length;
//   }

//   public boolean timerCompleted() {
//     if (timer - timerStarted == timerLength
//         && timerLength != 0) {
//       return true;
//     }
//     return false;
//   }

//   public void resetTimer() {
//     timerStarted = 0;
//     timerLength = 0;
//   }

// }
