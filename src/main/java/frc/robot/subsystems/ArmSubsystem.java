// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.ShooterState;

public class ArmSubsystem extends SubsystemBase {
  private final ShooterState m_shooterState;
  private final CANSparkMax m_armMotorMain;
  private final SparkPIDController m_armMainPIDController;
  private final RelativeEncoder m_MainEncoder;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  private static double kDt = 0.02; // 20ms (update rate for wpilib)
  private final double ksArmVolts = 0.08521;
  private final double kgArmGravityGain = 0.1711;
  private final double kvArmVoltSecondsPerMeter = 0.0022383;
  private final double kaArmVoltSecondsSquaredPerMeter = 0.00041162;
  private final double kMinArmAngleRadians = Units.degreesToRadians(Constants.ARMMINRELATVESTART);
  private final double kMaxArmAngleRadians = Units.degreesToRadians(Constants.ARMMAXRELATIVE);
  private final double kArmLoadAngleRadians =
      Units.degreesToRadians(Constants.ARMLOADANGLE); // angle to be when recieving ring
  private final double kArmSpeakerAngleRadians =
      Units.degreesToRadians(Constants.ARMSPEAKERANGLE); // angle to be when shooting into speaker
  private final double kArmAmpAngleRadians =
      Units.degreesToRadians(Constants.ARMAMPANGLE); // angle to be when shooting into amp
  private final double karmMaxVelocity = 1.0; // m/s
  private final double karmMaxAcceleration = 0.5; // m/s^2
  private boolean kPIDEnabled = true;
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kGearRatio = 120; // TBD, 48:1 / kGearRatio:1
  // basically converted from rotations to radians by multiplying by 2 pi, then adjusting for the
  // gear ratio by dividing by the gear ratio.
  // remember that 2pi radians in 360 degrees.
  private final double kRadiansConversionRatio = (Math.PI * 2) / kGearRatio;
  private final double kVelocityConversionRatio = kRadiansConversionRatio / 60;
  private final double kAbsoluteRadiansConversionRatio = (Math.PI * 2);
  private final double kAbsoluteVelocityConversionRatio = kAbsoluteRadiansConversionRatio / 60;
  private final ArmFeedforward m_armFeedforward =
      new ArmFeedforward(
          ksArmVolts, kgArmGravityGain, kvArmVoltSecondsPerMeter, kaArmVoltSecondsSquaredPerMeter);
  private final TrapezoidProfile m_armTrapezoidProfile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(karmMaxVelocity, karmMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private boolean m_stopped = true;

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(ShooterState shooterState) {
    m_shooterState = shooterState;
    // create the arm motors
    m_armMotorMain = new CANSparkMax(CANConstants.MOTORARMMAINID, CANSparkMax.MotorType.kBrushless);

    // set the idle mode to brake
    m_armMotorMain.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_armMotorMain.setInverted(true);

    // connect to built in PID controller
    m_armMainPIDController = m_armMotorMain.getPIDController();
    // allow us to read the encoder
    m_MainEncoder = m_armMotorMain.getEncoder();
    m_AbsoluteEncoder = m_armMotorMain.getAbsoluteEncoder();
    m_AbsoluteEncoder.setInverted(false);
    m_AbsoluteEncoder.setPositionConversionFactor(kAbsoluteRadiansConversionRatio);
    m_AbsoluteEncoder.setVelocityConversionFactor(kAbsoluteVelocityConversionRatio);
    // m_AbsoluteEncoder.setZeroOffset(Constants.ARMENCODEROFFSET);
    // setup the encoders
    m_MainEncoder.setPositionConversionFactor(kRadiansConversionRatio);
    m_MainEncoder.setVelocityConversionFactor(kVelocityConversionRatio);
    m_MainEncoder.setPosition(Units.degreesToRadians(Constants.ARMMINRELATVESTART));
    // PID coefficients
    kP = 1.7496;
    kI = 0;
    kD = 0.0068645;
    kIz = 0;
    kMaxOutput = 0.4;
    kMinOutput = -0.4;

    // set PID coefficients
    m_armMainPIDController.setP(kP);
    m_armMainPIDController.setI(kI);
    m_armMainPIDController.setD(kD);
    m_armMainPIDController.setIZone(kIz);
    m_armMainPIDController.setOutputRange(kMinOutput, kMaxOutput);
    m_armMainPIDController.setFeedbackDevice(m_AbsoluteEncoder);
    // m_armMotorMain.setSoftLimit(SoftLimitDirection.kReverse, kMinArmAngleRadians));
    // m_armMotorMain.setSoftLimit(SoftLimitDirection.kForward, kMaxArmAngleRadians);
    m_armMotorMain.burnFlash();

    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));
  }

  public void setVoltage(Measure<Voltage> voltage) {
    m_armMotorMain.setVoltage(voltage.in(Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public double getAverageEncoderPosition() {
    // get the average encoder position
    return (m_MainEncoder.getPosition());
  }

  /*
   * Move arm a certain number of radians relative to current position
   */
  public void MoveArmRelative(double radians) {
    radians = radians + getAverageEncoderPosition();
    // update the PID controller with current encoder position
    MoveArmToPosition(radians);
  }

  public void lowerArm() {
    // move to set lowered arm position
    MoveArmToPosition(kMinArmAngleRadians);
  }

  public void rasieArm() {
    // move to set raised arm position
    MoveArmToPosition(kMaxArmAngleRadians);
  }

  public void moveArmToLoad() {
    MoveArmToPosition(kArmLoadAngleRadians);
  }

  public void moveArmToAmp() {
    MoveArmToPosition(kArmAmpAngleRadians);
  }

  public void moveArmToSpeaker() {
    MoveArmToPosition(kArmSpeakerAngleRadians);
  }

  /*
   * Move arm to global position (by updating goal)
   */
  public void MoveArmToPosition(double radians) {
    m_stopped = false;
    // update the motion profile with new goal
    // add minimum starting angle to the target angle to get the real angle
    double final_radians = Math.max(radians, kMinArmAngleRadians);
    final_radians = Math.min(final_radians, kMaxArmAngleRadians);
    m_goal = new TrapezoidProfile.State(radians, 0); // set the goal
  }

  /*
   * attempt to hold arm at current location
   */
  public void stop() {
    if (!m_stopped) {
      // update the PID controller with current encoder position
      MoveArmToPosition(getAverageEncoderPosition());
      m_stopped = true;
    }
  }

  public void disablePID() {
    kPIDEnabled = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (kPIDEnabled) {
      // update the setpoint
      m_setpoint = m_armTrapezoidProfile.calculate(kDt, m_setpoint, m_goal);
      // Call the controller and feedforward with the target position and velocity
      m_armMainPIDController.setReference(
          m_setpoint.position,
          CANSparkBase.ControlType.kPosition,
          0,
          m_armFeedforward.calculate(m_setpoint.position, m_setpoint.velocity));
    }
    m_shooterState.updateDash();
    SmartDashboard.putNumber(
        "Current Arm Angle (Degrees) (Relative)",
        Units.radiansToDegrees(m_MainEncoder.getPosition()));
    SmartDashboard.putNumber(
        "Current Arm Angle (Degrees) (Absolute)",
        Units.radiansToDegrees(m_AbsoluteEncoder.getPosition()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
