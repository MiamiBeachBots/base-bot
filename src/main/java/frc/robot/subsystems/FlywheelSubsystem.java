// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;

public class FlywheelSubsystem extends SubsystemBase {
  private final SparkMax m_ShooterMotorMain;
  private final SparkMax m_ShooterMotorSecondary;

  private final SparkMaxConfig m_MainConfig = new SparkMaxConfig(); // Motor Configuration
  private final SparkMaxConfig m_SecondaryConfig = new SparkMaxConfig(); // Motor Configuration

  private final SparkClosedLoopController m_ShooterMainPIDController;
  private RelativeEncoder m_ShooterMainEncoder;
  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kWheelDiameter = Units.inchesToMeters(4); // meters
  private final double kGearRatio = 4; // TBD
  // basically converted from rotations to to radians to then meters using the wheel diameter.
  // the diameter is already *2 so we don't need to multiply by 2 again.
  private final double kPositionConversionRatio = (Math.PI * kWheelDiameter) / kGearRatio;
  private final double kVelocityConversionRatio = kPositionConversionRatio / 60;

  // setup feedforward
  private final double ksShooterVolts = 0.2063;
  private final double kvDriveVoltSecondsPerMeter = 1.5611;
  private final double kaDriveVoltSecondsSquaredPerMeter = 0.1396;

  SimpleMotorFeedforward m_shooterFeedForward =
      new SimpleMotorFeedforward(
          ksShooterVolts, kvDriveVoltSecondsPerMeter, kaDriveVoltSecondsSquaredPerMeter);

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  // current limit
  private final int k_CurrentLimit = 80;

  /** Creates a new ShooterSubsystem. */
  public FlywheelSubsystem() {
    // create the shooter motors
    m_ShooterMotorMain =
        new SparkMax(CANConstants.MOTORSHOOTERLEFTID, SparkMax.MotorType.kBrushless);
    m_ShooterMotorSecondary =
        new SparkMax(CANConstants.MOTORSHOOTERRIGHTID, SparkMax.MotorType.kBrushless);

    // set the idle mode to coast
    m_MainConfig.idleMode(IdleMode.kBrake);
    m_SecondaryConfig.idleMode(IdleMode.kBrake);
    // set current limit
    m_MainConfig.smartCurrentLimit(k_CurrentLimit);
    m_SecondaryConfig.smartCurrentLimit(k_CurrentLimit);
    // config follow
    m_SecondaryConfig.follow(m_ShooterMotorMain, true);

    // connect to built in PID controller
    m_ShooterMainPIDController = m_ShooterMotorMain.getClosedLoopController();

    // allow us to read the encoder
    m_ShooterMainEncoder = m_ShooterMotorMain.getEncoder();
    m_MainConfig.encoder.positionConversionFactor(kPositionConversionRatio);
    m_MainConfig.encoder.velocityConversionFactor(kVelocityConversionRatio);
    // PID coefficients
    kP = 0.00013373;
    kI = 0;
    kD = 0;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    // set PID coefficients
    m_MainConfig.closedLoop.pid(kP, kI, kD, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_MainConfig.closedLoop.iZone(kIz, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_MainConfig.closedLoop.outputRange(
        kMinOutput, kMaxOutput, DriveConstants.kDrivetrainVelocityPIDSlot);
    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));
    m_ShooterMotorMain.configure(
        m_MainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_ShooterMotorSecondary.configure(
        m_SecondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVoltage(Voltage voltage) {
    m_ShooterMotorMain.setVoltage(voltage.in(Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /*
   * Spin shooter at a given Speed (M/S)
   */
  public void SpinShooter(double speed) {
    m_ShooterMainPIDController.setReference(
        speed,
        SparkBase.ControlType.kVelocity,
        DriveConstants.kDrivetrainVelocityPIDSlot,
        m_shooterFeedForward.calculate(speed));
  }

  public void SpinAtFull() {
    m_ShooterMotorMain.set(1);
  }

  /*
   * Stop the shooter
   */
  public void StopShooter() {
    SpinShooter(0);
  }

  /*
   * Check if shooter is at a given Speed
   */
  public Boolean isAtSpeedTolerance(double speed) {
    return (m_ShooterMainEncoder.getVelocity() > speed - 0.1
        && m_ShooterMainEncoder.getVelocity() < speed + 0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Motor Speed m/s", m_ShooterMainEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
