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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  // int smartMotionSlot = 0;
  // int allowedErr;
  // int minVel;
  // double kP = 0;
  // double kI = 0;
  // double kD = 0;
  // double kIz = 0;
  // double kFF = 0.000156;
  // double kMaxOutput = 1;
  // double kMinOutput = -1;
  // double maxRPM = 5700;
  // double maxVel = 2000;
  // double maxAcc = 1500;
  // double setPointDrive = 0;

  // public DoubleSolenoid leftOuterArmPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 1);
  // public DoubleSolenoid leftInnerArmPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
  // public DoubleSolenoid rightOuterPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 8, 9);
  // public DoubleSolenoid rightInnerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 10, 11);

  // public CANSparkMax leftOuterMotor = new CANSparkMax(9, MotorType.kBrushless);
  // public CANSparkMax leftInnerMotor = new CANSparkMax(10, MotorType.kBrushless);
  // public CANSparkMax rightOuterMotor = new CANSparkMax(11, MotorType.kBrushless);
  // public CANSparkMax rightInnerMotor = new CANSparkMax(12, MotorType.kBrushless);

  // public RelativeEncoder m_leftOuterEncoder = leftOuterMotor.getEncoder();
  // public RelativeEncoder m_leftInnerEncoder = leftInnerMotor.getEncoder();
  // public RelativeEncoder m_rightOuterEncoder = rightOuterMotor.getEncoder();
  // public RelativeEncoder m_rightInnerEncoder = rightInnerMotor.getEncoder();

  // public SparkMaxPIDController m_leftOuterController = leftOuterMotor.getPIDController();
  // public SparkMaxPIDController m_leftInnerController = leftInnerMotor.getPIDController();
  // public SparkMaxPIDController m_rightOuterController = rightOuterMotor.getPIDController();
  // public SparkMaxPIDController m_rightInnerController = rightInnerMotor.getPIDController();

  // int climbingStage = 0;

  // int timer = 0;

  // int timerStarted = 0;

  // int timerLength = 0;

  // enum ArmExtendedStates {
  //   UP, DOWN
  // }

  // /* Creates a new ClimberSubsystem. */
  // public ClimberSubsystem() {
  //   initializePID(m_leftOuterController, m_leftOuterEncoder);
  //   initializePID(m_leftInnerController, m_leftInnerEncoder);
  //   initializePID(m_rightOuterController, m_rightOuterEncoder);
  //   initializePID(m_rightInnerController, m_rightInnerEncoder);
  // }

  // public void initializePID(SparkMaxPIDController p, RelativeEncoder h) {
  //   p.setP(kP);
  //   p.setI(kI);
  //   p.setD(kD);
  //   p.setIZone(kIz);
  //   p.setFF(kFF);
  //   p.setOutputRange(kMinOutput, kMaxOutput);
  //   p.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
  //   p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
  //   p.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
  //   p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  //   h.setPositionConversionFactor(1);
  //   h.setVelocityConversionFactor(1);
  // }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }

  // public void updateTimer() {
  //   timer += 1; // Update every 20ms
  // }

  // public void doClimbCycle() {
  //   double leftOuterPosition = m_leftOuterEncoder.getPosition();
  //   double leftInnerPosition = m_leftInnerEncoder.getPosition();
  //   double rightOuterPosition = m_rightOuterEncoder.getPosition();
  //   double rightInnerPosition = m_rightInnerEncoder.getPosition();

  //   switch (climbingStage) {
  //     case 0:
  //       /*
  //        * At the end of this stage the outer arms are extended to reach the second bar
  //        */
  //       oldToggleOuterArms();
  //       startTimer(Constants.timerDelayBetweenSteps);
  //       climbingStage = 1;
  //       break;
  //     case 1:
  //       /*
  //        * At the end of this stage the robot is moved forward to align with the bar
  //        */
  //       if (timerCompleted()) {
  //         resetTimer();
  //         // TODO - harass jason to add a drive thing
  //         startTimer(Constants.timerDelayBetweenSteps);
  //         climbingStage = 2;
  //       }
  //       break;
  //     case 2:
  //       /*
  //        * At the end of this stage the outer arms are retracted and the robot is
  //        * hanging on the 2nd bar
  //        */
  //       if (timerCompleted()) {
  //         resetTimer();
  //         oldToggleOuterArms();
  //         startTimer(Constants.timerDelayBetweenSteps);
  //         climbingStage = 3;
  //       }
  //       break;
  //     case 3:
  //       /*
  //        * At the end of this stage we start moving the inner arms towards the 3rd bar
  //        */
  //       if (timerCompleted()) {
  //         resetTimer();
  //         setInnerArmsPosition(Constants.firstExtendSetPoint);
  //         startTimer(Constants.timerDelayBetweenSteps);
  //         climbingStage = 4;
  //       }
  //       break;
  //     case 4:
  //       /*
  //        * At the end of this stage we start moving the inner arms to be aligned with
  //        * the bar
  //        */
  //       if (leftInnerPosition == Constants.firstExtendSetPoint
  //           && timerCompleted()) {
  //         resetTimer();
  //         oldToggleInnerArms();
  //         setInnerArmsPosition(Constants.barAlignedSetPoint);
  //         startTimer(Constants.timerDelayBetweenSteps);
  //         climbingStage = 5;
  //       }
  //       break;
  //     case 5:
  //       /*
  //        * At the end of this stage the Inner arms are on bar 3 and we start moving the
  //        * outer ones towards bar 4
  //        */
  //       if (leftInnerPosition == Constants.barAlignedSetPoint
  //           && timerCompleted()) {
  //         resetTimer();
  //         oldToggleInnerArms();
  //         /* There may be a delay here */
  //         startTimer(Constants.pneumaticTimerDelay);
  //         climbingStage = 6;
  //       }
  //       break;
  //     case 6:
  //       if (timerCompleted()) {
  //         resetTimer();
  //         oldToggleOuterArms();
  //         setOuterArmsPosition(Constants.firstExtendSetPoint);
  //         startTimer(Constants.timerDelayBetweenSteps);
  //         climbingStage = 7;
  //       }
  //       break;
  //     case 7:
  //       /*
  //        * At the end of this stage we start moving the outer arms towards the
  //        * transversal bar
  //        */
  //       if (leftOuterPosition == Constants.firstExtendSetPoint
  //           && leftOuterArmPneumatic.get() == Value.kReverse
  //           && timerCompleted()) {
  //         resetTimer();
  //         oldToggleOuterArms();
  //         setOuterArmsPosition(Constants.barAlignedSetPoint);
  //         startTimer(Constants.timerDelayBetweenSteps);
  //         climbingStage = 8;
  //       } else if (leftOuterArmPneumatic.get() == Value.kForward) {
  //         oldToggleOuterArms();
  //       }
  //       break;
  //     case 8:
  //       /*
  //        * At the end of this stage we get the outer arms on the transversal bar and
  //        * release the inner
  //        */
  //       if (leftOuterPosition == Constants.barAlignedSetPoint
  //           && timerCompleted()) {
  //         resetTimer();
  //         oldToggleOuterArms();
  //         /* There may be a delay here */
  //         startTimer(Constants.pneumaticTimerDelay);
  //         climbingStage = 9;
  //       }
  //       break;
  //     case 9:
  //       if (timerCompleted()) {
  //         resetTimer();
  //         oldToggleInnerArms();
  //         startTimer(Constants.timerDelayBetweenSteps);
  //         climbingStage = 10;
  //       }
  //       break;
  //     case 10:
  //       if (leftInnerPosition == Constants.firstExtendSetPoint
  //           && leftInnerArmPneumatic.get() == Value.kReverse
  //           && timerCompleted()) {
  //         resetTimer();
  //         oldToggleInnerArms();
  //         setInnerArmsPosition(Constants.firstExtendSetPoint);
  //         climbingStage = 16;
  //       } else if (leftInnerArmPneumatic.get() == Value.kForward) {
  //         oldToggleInnerArms();
  //       }
  //     default:
  //       /* If this function gets called then we're screwed */
  //       break;
  //   }
  // }

  // public void setOuterArmsPosition(double position) {
  //   m_leftOuterController.setReference(position * 1.0, CANSparkMax.ControlType.kSmartMotion);
  //   m_rightOuterController.setReference(position * 1.0, CANSparkMax.ControlType.kSmartMotion);
  // }

  // public void setInnerArmsPosition(double position) {
  //   m_leftInnerController.setReference(position * 1.0, CANSparkMax.ControlType.kSmartMotion);
  //   m_rightInnerController.setReference(position * 1.0, CANSparkMax.ControlType.kSmartMotion);
  // }

  // public void toggleOuterArms(ArmExtendedStates state) {
  //   switch (state) {
  //     case UP:
  //       break;
  //     case DOWN:
  //       break;
  //     default:
  //       break;
  //   }
  // }

  // public void oldToggleOuterArms() { // Reverses the toggle state of the outer solenoids
  //   if (leftOuterArmPneumatic.get() == Value.kForward) {
  //     leftOuterArmPneumatic.set(Value.kReverse);
  //     rightOuterPneumatic.set(Value.kReverse);
  //   } else if (leftOuterArmPneumatic.get() == Value.kReverse) {
  //     leftOuterArmPneumatic.set(Value.kForward);
  //     rightOuterPneumatic.set(Value.kForward);
  //   }
  // }

  // public void oldToggleInnerArms() { // Reverses the toggle state of the inner solenoids
  //   if (leftInnerArmPneumatic.get() == Value.kForward) {
  //     leftInnerArmPneumatic.set(Value.kReverse);
  //     rightInnerPneumatic.set(Value.kReverse);
  //   } else if (leftInnerArmPneumatic.get() == Value.kReverse) {
  //     leftInnerArmPneumatic.set(Value.kForward);
  //     rightInnerPneumatic.set(Value.kForward);
  //   }
  // }

  // public double getArcLength() {
  //   double arcLength = (Constants.hookLengthToBase / Constants.climbingArmLength) * Constants.climbingArmLength;
  //   return arcLength;
  // }

  // public double getNumberOfClicks() {
  //   double numberOfClicks = getArcLength() / Constants.distancePerMotorClick;
  //   return numberOfClicks;
  // }

  // public void startTimer(int length) {
  //   timerStarted = timer;
  //   timerLength = length;
  // }

  // public boolean timerCompleted() {
  //   if (timer - timerStarted == timerLength
  //       && timerLength != 0) {
  //     return true;
  //   }
  //   return false;
  // }

  // public void resetTimer() {
  //   timerStarted = 0;
  //   timerLength = 0;
  // }
}
