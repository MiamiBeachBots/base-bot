// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotorMain;
  private final SparkPIDController m_armMainPIDController;
  private final RelativeEncoder m_MainEncoder;
  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  private final double kminArmAngle =
      Units.degreesToRadians(
          0); // this is needed for the feedforward control, basically min angle relative to flat on
  // floor.
  private static double kDt = 0.02; // 20ms (update rate for wpilib)
  private final double ksArmVolts = 0.0;
  private final double kgArmGravityGain = 0.0;
  private final double kvArmVoltSecondsPerMeter = 0.0;
  private final double kaArmVoltSecondsSquaredPerMeter = 0.0;
  private final double kLoweredArmPositionRadians = Units.degreesToRadians(45);
  private final double karmMaxVelocity = 2; // m/s
  private final double karmMaxAcceleration = 1; // m/s^2
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kGearRatio = 1; // TBD
  // basically converted from rotations to radians by multiplying by 2 pi, then adjusting for the
  // gear ratio by dividing by the gear ratio.
  // remember that 2pi radians in 360 degrees.
  private final double kRadiansConversionRatio = (Math.PI * 2) / kGearRatio;
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
  public ArmSubsystem() {
    // create the arm motors
    m_armMotorMain = new CANSparkMax(CANConstants.MOTORARMMAINID, CANSparkMax.MotorType.kBrushless);

    // set the idle mode to brake
    m_armMotorMain.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // connect to built in PID controller
    m_armMainPIDController = m_armMotorMain.getPIDController();
    // allow us to read the encoder
    m_MainEncoder = m_armMotorMain.getEncoder();
    // setup the encoders
    m_MainEncoder.setPositionConversionFactor(kRadiansConversionRatio);
    // PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_armMainPIDController.setP(kP);
    m_armMainPIDController.setI(kI);
    m_armMainPIDController.setD(kD);
    m_armMainPIDController.setIZone(kIz);
    m_armMainPIDController.setOutputRange(kMinOutput, kMaxOutput);

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
    MoveArmToPosition(kLoweredArmPositionRadians);
  }

  /*
   * Move arm to global position (by updating goal)
   */
  public void MoveArmToPosition(double radians) {
    m_stopped = false;
    // update the motion profile with new goal
    // add minimum starting angle to the target angle to get the real angle
    double total_radians = radians + kminArmAngle;
    m_goal = new TrapezoidProfile.State(total_radians, 0); // set the goal
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

  public void zeroEncoders() {
    m_MainEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update the setpoint
    m_setpoint = m_armTrapezoidProfile.calculate(kDt, m_setpoint, m_goal);

    // Call the controller and feedforward with the target position and velocity
    m_armMainPIDController.setReference(
        m_setpoint.position,
        CANSparkBase.ControlType.kPosition,
        0,
        m_armFeedforward.calculate(m_setpoint.position, m_setpoint.velocity));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
