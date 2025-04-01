package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;
import frc.robot.utils.HelperFunctions;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  // Decalare Motor
  private final SparkMax m_Motor;
  // Declare Simulated Motor
  private final DCMotor m_simGearbox;
  private final SparkMaxSim m_simMotor;

  // Declare Motor Configs
  private final SparkMaxConfig m_MotorConfig = new SparkMaxConfig();
  // Declare PID
  private final SparkClosedLoopController m_ArmMainPIDController;
  // Declare Encoders
  private RelativeEncoder m_ArmEncoder;
  private AbsoluteEncoder m_ArmAbsoluteEncoder;
  // Declare Simulated Encoder
  private final SparkRelativeEncoderSim m_ArmEncoderSim;
  private final SparkAbsoluteEncoderSim m_ArmAbsoluteEncoderSim;
  // Declare Arm Physics Engine
  private final SingleJointedArmSim m_ArmSim;

  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kGearRatio = 64; // TBD
  // basically converted from rotations to to radians to then meters using the wheel diameter.
  // the diameter is already *2 so we don't need to multiply by 2 again.
  private final double kPositionConversionRatio = (Math.PI * 2) / kGearRatio;
  private final double kVelocityConversionRatio = kPositionConversionRatio / 60;

  private final double kPositionConversionRatioAbsolute = (Math.PI * 2);
  private final double kVelocityConversionRatioAbsolute = kPositionConversionRatioAbsolute / 60;

  // setup feedforward
  private final double kS = 0.64053; // Static Friction (Volts)
  private final double kG = 0.64527; // Inertia (Volts)
  private final double kV = 0.78309; // Mass Volts/(rad/s)
  private final double kA = 0.27366; // Acceleration Volts/(rad/s^2)

  // other constants
  private final double kMinAngleRads = Constants.ARM_START_OFFSET;
  private final double kMaxAngleRads = kMinAngleRads + Constants.ARM_ANGLE_OFFSET;
  private final double kArmLengthMeters = 0.1;
  private final double kjKgMetersSquared =
      0.1; // The moment of inertia of the arm; can be calculated from CAD software.

  ArmFeedforward m_ArmFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  // setup trapezoidal motion profile
  private final double kMaxVelocity = Units.degreesToRadians(45); // R/S
  private final double kMaxAcceleration = Units.degreesToRadians(30); // R/S^2
  private final double kAllowedClosedLoopError = 0.35; // Radians (about 2 degrees)

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  // current limit
  private final int k_CurrentLimit = 40;

  // Requested Angle
  private double m_requestedAngle = 0;

  // disable PID when profiling
  private boolean m_PIDEnabled = true;

  public ArmSubsystem() {
    // Create Arm motor
    m_Motor = new SparkMax(CANConstants.MOTOR_ARM_MAIN_ID, SparkMax.MotorType.kBrushless);

    // Create Simulated Motors
    m_simGearbox = DCMotor.getNEO(1);
    m_simMotor = new SparkMaxSim(m_Motor, m_simGearbox);

    // Create Simulated encoder
    m_ArmEncoderSim = m_simMotor.getRelativeEncoderSim();
    m_ArmAbsoluteEncoderSim = m_simMotor.getAbsoluteEncoderSim();

    // Create Simulated Physics Engine
    m_ArmSim =
        new SingleJointedArmSim(
            m_simGearbox,
            kGearRatio,
            kjKgMetersSquared,
            kArmLengthMeters,
            kMinAngleRads,
            kMaxAngleRads,
            true,
            Constants.ARM_ANGLE_OFFSET,
            0.01,
            0.001);

    // Set idle mode to coast
    m_MotorConfig.idleMode(IdleMode.kBrake);
    // Set current limit
    m_MotorConfig.smartCurrentLimit(k_CurrentLimit);

    // invert direction
    m_MotorConfig.inverted(true);
    m_MotorConfig.absoluteEncoder.inverted(true);

    // Connect to built in PID controller
    m_ArmMainPIDController = m_Motor.getClosedLoopController();

    // Allow us to read the encoders
    m_ArmEncoder = m_Motor.getEncoder();
    m_ArmAbsoluteEncoder = m_Motor.getAbsoluteEncoder();

    // Set Conversion Factors
    m_MotorConfig.encoder.positionConversionFactor(kPositionConversionRatio);
    m_MotorConfig.encoder.velocityConversionFactor(kVelocityConversionRatio);
    m_MotorConfig.absoluteEncoder.positionConversionFactor(kPositionConversionRatioAbsolute);
    m_MotorConfig.absoluteEncoder.velocityConversionFactor(kVelocityConversionRatioAbsolute);

    // set absolute encoder zero offset
    m_MotorConfig.absoluteEncoder.zeroOffset(Constants.ARM_ZERO_ENCODER_OFFSET);

    // PID coefficients
    kP = 0.64545;
    kI = 0;
    kD = 0.25;
    kIz = 0;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;
    // set PID coefficients
    m_MotorConfig.closedLoop.pid(kP, kI, kD, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.iZone(kIz, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.outputRange(
        kMinOutput, kMaxOutput, DriveConstants.kDrivetrainPositionPIDSlot);
    // use absolute encoder for pid
    m_MotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // Smart Control Config
    m_MotorConfig.closedLoop.maxMotion.maxVelocity(
        kMaxVelocity, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.maxMotion.maxAcceleration(
        kMaxAcceleration, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.maxMotion.allowedClosedLoopError(
        kAllowedClosedLoopError, DriveConstants.kDrivetrainPositionPIDSlot);

    // set soft limits
    m_MotorConfig.softLimit.forwardSoftLimitEnabled(true);
    m_MotorConfig.softLimit.forwardSoftLimit(kMaxAngleRads);
    m_MotorConfig.softLimit.reverseSoftLimitEnabled(true);
    m_MotorConfig.softLimit.reverseSoftLimit(kMinAngleRads);

    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));

    m_Motor.configure(
        m_MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SetAngle(Constants.ARM_ANGLE_OFFSET); // Set arm initial goal to fully up
  }

  public void setVoltage(Voltage voltage) {
    m_Motor.setVoltage(voltage.in(Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * Move Arm to a specific angle
   *
   * @param radians Angle in radians to move the arm to
   */
  public void SetAngle(double radians) {
    m_requestedAngle = radians + kMinAngleRads; // 90 degree down offset
    m_goal = new TrapezoidProfile.State(m_requestedAngle, 0);
  }

  public double GetAngle() {
    return m_ArmAbsoluteEncoder.getPosition();
  }

  public boolean atGoal() {
    return HelperFunctions.inRange(m_requestedAngle, GetAngle(), kAllowedClosedLoopError);
  }

  /** Lower the Arm */
  public void LowerArm() {
    SetAngle(kMinAngleRads);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("ArmStartingOffsetDegrees", Units.radiansToDegrees(kMinAngleRads));
    Logger.recordOutput("ArmAbsolutePositionRadians", m_ArmAbsoluteEncoder.getPosition());
    Logger.recordOutput("ArmAbsoluteVelocityRPM", m_ArmAbsoluteEncoder.getVelocity());
    Logger.recordOutput("ArmRequestedAngle", m_requestedAngle);
    Logger.recordOutput(
        "ArmRequestedAngleDegreesWO", Units.radiansToDegrees(m_requestedAngle - kMinAngleRads));
    Logger.recordOutput(
        "ArmAbsoluteEnoderDegreesWO",
        Units.radiansToDegrees(m_ArmAbsoluteEncoder.getPosition() - kMinAngleRads));
    m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
    if (m_PIDEnabled) {
      m_ArmMainPIDController.setReference(
          m_setpoint.position,
          SparkBase.ControlType.kPosition,
          DriveConstants.kDrivetrainPositionPIDSlot,
          m_ArmFeedforward.calculate(m_setpoint.position, m_setpoint.velocity));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Update the simulation of our Arm, set inputs
    m_ArmSim.setInput(m_simMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // update simulation (20ms)
    m_ArmSim.update(0.020);

    // Iterate PID loops
    m_simMotor.iterate(m_ArmSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

    // add load to battery
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_ArmSim.getCurrentDrawAmps()));

    // update encoder
    m_ArmEncoderSim.setPosition(m_ArmSim.getAngleRads());
    m_ArmEncoderSim.setVelocity(m_ArmSim.getVelocityRadPerSec());
    m_ArmAbsoluteEncoderSim.setPosition(m_ArmSim.getAngleRads());
    m_ArmAbsoluteEncoderSim.setVelocity(m_ArmSim.getVelocityRadPerSec());
  }

  public void disablePID() {
    m_PIDEnabled = false;
  }
}
