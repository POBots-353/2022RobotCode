// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class ClimberSubsystem extends SubsystemBase {

//   int smartMotionSlot = 0;
//   int allowedErr;
//   int minVel;
//   double kP = 0;
//   double kI = 0;
//   double kD = 0;
//   double kIz = 0;
//   double kFF = 0.000156;
//   double kMaxOutput = 1;
//   double kMinOutput = -1;
//   double maxRPM = 5700;
//   double maxVel = 2000;
//   double maxAcc = 1500;
//   double setPointDrive = 0;

//   // public DoubleSolenoid outerArmPneumatic = new
//   // DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 1); // PORT NUMBERS ARE
//   // UNKNOWN AT THIS
//   // TIME

//   public DoubleSolenoid leftOuterPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
//   public DoubleSolenoid leftInnerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 1);
//   public DoubleSolenoid rightOuterPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 1);
//   public DoubleSolenoid rightInnerPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 8, 1);

//   public CANSparkMax leftOuterMotor = new CANSparkMax(9, MotorType.kBrushless);
//   public CANSparkMax leftInnerMotor = new CANSparkMax(10,
//       MotorType.kBrushless);
//   public CANSparkMax rightOuterMotor = new CANSparkMax(11,
//       MotorType.kBrushless);
//   public CANSparkMax rightInnerMotor = new CANSparkMax(12,
//       MotorType.kBrushless);

//   public RelativeEncoder m_leftOuterEncoder = leftOuterMotor.getEncoder();
//   public RelativeEncoder m_leftInnerEncoder = leftInnerMotor.getEncoder();
//   public RelativeEncoder m_rightOuterEncoder = rightOuterMotor.getEncoder();
//   public RelativeEncoder m_rightInnerEncoder = rightInnerMotor.getEncoder();

//   public SparkMaxPIDController m_leftOuterController = leftOuterMotor.getPIDController();
//   public SparkMaxPIDController m_leftInnerController = leftInnerMotor.getPIDController();
//   public SparkMaxPIDController m_rightOuterController = rightOuterMotor.getPIDController();
//   public SparkMaxPIDController m_rightInnerController = rightInnerMotor.getPIDController();

//   public double currentOuterReferencePoint = 0;
//   public double currentInnerReferencePoint = 0;

//   enum ArmExtendedStates {
//     UP, DOWN
//   }

//   /* Creates a new ClimberSubsystem. */
//   public ClimberSubsystem() {
//     initializePID(m_leftOuterController, m_leftOuterEncoder);
//     initializePID(m_leftInnerController, m_leftInnerEncoder);
//     initializePID(m_rightOuterController, m_rightOuterEncoder);
//     initializePID(m_rightInnerController, m_rightInnerEncoder);
//   }

//   public void initializePID(SparkMaxPIDController p, RelativeEncoder h) {
//     p.setP(kP);
//     p.setI(kI);
//     p.setD(kD);
//     p.setIZone(kIz);
//     p.setFF(kFF);
//     p.setOutputRange(kMinOutput, kMaxOutput);
//     p.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
//     p.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
//     p.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
//     p.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
//     h.setPositionConversionFactor(1);
//     h.setVelocityConversionFactor(1);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public void setOuterArmsPosition(double position) {
//     m_leftOuterController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
//     m_rightOuterController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
//     currentOuterReferencePoint = position;
//   }

//   public void setInnerArmsPosition(double position) {
//     m_leftInnerController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
//     m_rightInnerController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
//     currentInnerReferencePoint = position;
//   }

//   public void toggleOuterArms(ArmExtendedStates state) {
//     switch (state) {
//       case UP:
//         break;
//       case DOWN:
//         break;
//       default:
//         break;
//     }
//   }

//   public void oldToggleOuterArms() { // Reverses the toggle state of the outer solenoids
//     leftOuterPneumatic.toggle();
//     rightOuterPneumatic.toggle();
//   }

//   public void oldToggleInnerArms() { // Reverses the toggle state of the inner solenoids
//     leftInnerPneumatic.toggle();
//     rightInnerPneumatic.toggle();
//   }

//   public double getArcLength() {
//     double arcLength = (Constants.hookLengthToBase / Constants.climbingArmLength) * Constants.climbingArmLength;
//     return arcLength;
//   }

//   public double getNumberOfClicks() {
//     double numberOfClicks = getArcLength() / Constants.distancePerMotorClick;
//     return numberOfClicks;
//   }
// }
