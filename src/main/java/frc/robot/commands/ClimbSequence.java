// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.ClimberSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ClimbSequence extends SequentialCommandGroup {
//   ClimberSubsystem climberSubsystem;

//   Command ToggleOuterArms() {
//     return new RunCommand(() -> climberSubsystem.toggleOuterArms(), climberSubsystem);
//   }

//   Command ToggleInnerArms() {
//     return new RunCommand(() -> climberSubsystem.toggleInnerArms(), climberSubsystem);
//   }

//   Command MoveOuterArms(double position) {
//     return new RunCommand(() -> climberSubsystem.setOuterArmsPosition(position), climberSubsystem)
//         .withInterrupt(climberSubsystem::outerMoveFinished);
//   }

//   Command Wait(double seconds) {
//     return new WaitCommand(seconds);
//   }

//   Command DisablePID() {
//     return new RunCommand(() -> climberSubsystem.disablePID(), climberSubsystem);
//   }

//   Command EnablePID(double position) {
//     return new RunCommand(() -> {
//       climberSubsystem.setOuterPID(true);
//       climberSubsystem.setOuterArmsPosition(position);
//     }, climberSubsystem);
//   }

//   /** Creates a new ClimbSequence. */
//   public ClimbSequence(ClimberSubsystem climber) {
//     climberSubsystem = climber;
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//         /*
//          * Outer: Extended
//          * Inner: Retracted
//          */
//         ToggleOuterArms(),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Extended
//          * Inner: Extended
//          */
//         ToggleInnerArms(),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Retracted
//          * Inner: Extended
//          */
//         ToggleInnerArms(),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Retracted
//          * Inner: Retracted
//          */
//         ToggleInnerArms(),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Extended, Position Between 90 and Next Bar
//          * Inner: Retracted
//          */
//         MoveOuterArms(Constants.firstExtendSetPoint),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Retracted, Position Between 90 and Next Bar
//          * Inner: Retracted
//          */
//         ToggleOuterArms(),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Retracted, Behind the next bar
//          * Inner: Retracted
//          */
//         MoveOuterArms(Constants.behindBarSetPoint),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Extended, Behind the next bar
//          * Inner: Retracted
//          */
//         ToggleOuterArms(),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Extended, Aligned with the next bar
//          * Inner: Retracted
//          */
//         MoveOuterArms(Constants.barAlignedSetPoint),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Retracted, On the next Bar (PID DISABLED)
//          * Inner: Retracted
//          */
//         ToggleOuterArms(),
//         Wait(Constants.MLGWaterBucketClutchSeconds),
//         /*
//          * MLG WATER BUCKET CLUTCH
//          * We disable PID at the right time to prevent stuff from breaking (probably
//          * gonna break anyway but it won't be my fault)
//          */
//         DisablePID(),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Retracted, Unknown Angle
//          * Inner: Retracted
//          */
//         EnablePID(climberSubsystem.outerEncoder.getPosition()),
//         Wait(Constants.secondsDelayBetweenSteps),
//         /*
//          * Outer: Retracted, Unknown Angle
//          * Inner: Extended
//          */
//         ToggleInnerArms()
//     /*
//      * I have no idea.
//      */
//     );
//   }
// }