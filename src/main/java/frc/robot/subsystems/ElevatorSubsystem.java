package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Declare Motor
  private final SparkMax m_Motor;
  // Declare Simulated Motor
  private final DCMotor m_simGearbox;
  private final SparkMaxSim m_simMotor;

  // Instantiate Motor Configs
  private final SparkMaxConfig m_MotorConfig = new SparkMaxConfig();
  // Declare PID
  private final SparkClosedLoopController m_ElevatorMainPIDController;
  // Declare Encoder
  private RelativeEncoder m_ElevatorEncoder;
  // Declare Simulated Encoder
  private final SparkRelativeEncoderSim m_ElevatorEncoderSim;
  // Declare Elevator Physics Engine
  private final ElevatorSim m_ElevatorSim;

  // TODO: Update to accurate values
  private final double kP, kI, kD, kIz, kMaxOutput, kMinOutput;
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kWheelDiameter = 1; // meters TODO: Figure out ratio
  private final double kGearRatio = 16; // TBD
  // basically converted from rotations to to radians to then meters using the wheel diameter.
  // the diameter is already *2 so we don't need to multiply by 2 again.
  private final double kPositionConversionRatio = (Math.PI * kWheelDiameter) / kGearRatio;
  private final double kVelocityConversionRatio = kPositionConversionRatio / 60;

  // setup feedforward
  private final double kS = 0.1; // Static Friction (Volts)
  private final double kG = 0.1; // Inertia (Volts)
  private final double kV = 0.1; // Mass (Volts*Seconds / Meter)
  private final double kA = 0.1; // Acceleration (Volts * Seconds^2 / Meter)

  // other constants
  private final double kMaxHeightMeters = 1.0; // TODO: Update
  private final double kMinHeightMeters = 0.0; // TODO: Update
  private final double kStartingHeightMeters = kMinHeightMeters + 0.1; // TODO: Update

  ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  // setup trapezoidal motion profile
  private final double kMaxVelocity = 0.1; // M/S TODO: Update
  private final double kMaxAcceleration = 0.01; // M/S^2 TODO: Update
  private final double kAllowedClosedLoopError = 0.01; // Meters

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  // current limit
  private final int k_CurrentLimit = 60;

  public ElevatorSubsystem() {
    // Create elevator motor
    m_Motor = new SparkMax(CANConstants.MOTOR_ELEVATOR_ID, SparkMax.MotorType.kBrushless);

    // Create Simulated Motors
    m_simGearbox = DCMotor.getNEO(1);
    m_simMotor = new SparkMaxSim(m_Motor, m_simGearbox);

    // Create Simulated encoder
    m_ElevatorEncoderSim = m_simMotor.getRelativeEncoderSim();

    // Create Simulated Physics Engine
    m_ElevatorSim =
        new ElevatorSim(
            kV,
            kA,
            m_simGearbox,
            kMinHeightMeters,
            kMaxHeightMeters,
            true,
            kStartingHeightMeters,
            0.01,
            0.0);

    // Set idle mode to coast
    m_MotorConfig.idleMode(IdleMode.kBrake);
    // Set current limit
    m_MotorConfig.smartCurrentLimit(k_CurrentLimit);

    // Connect to built in PID controller
    m_ElevatorMainPIDController = m_Motor.getClosedLoopController();

    // Allow us to read the encoder
    m_ElevatorEncoder = m_Motor.getEncoder();

    m_MotorConfig.encoder.positionConversionFactor(kPositionConversionRatio);
    m_MotorConfig.encoder.velocityConversionFactor(kVelocityConversionRatio);

    // PID coefficients
    kP = 0.0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kMaxOutput = 0.8;
    kMinOutput = -0.8;
    // set PID coefficients
    m_MotorConfig.closedLoop.pid(kP, kI, kD, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.iZone(kIz, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.outputRange(
        kMinOutput, kMaxOutput, DriveConstants.kDrivetrainPositionPIDSlot);
    // Smart Control Config
    m_MotorConfig.closedLoop.maxMotion.maxVelocity(
        kMaxVelocity, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.maxMotion.maxAcceleration(
        kMaxAcceleration, DriveConstants.kDrivetrainPositionPIDSlot);
    m_MotorConfig.closedLoop.maxMotion.allowedClosedLoopError(
        kAllowedClosedLoopError, DriveConstants.kDrivetrainPositionPIDSlot);
    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage),
                null, // No log consumer, since data is recorded by URCL
                this));

    m_Motor.configure(
        m_MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  /*
   * Move elevator to a specific height
   */
  public void SetHeight(double meters) {
    m_ElevatorMainPIDController.setReference(
        meters,
        SparkBase.ControlType.kMAXMotionPositionControl,
        DriveConstants.kDrivetrainPositionPIDSlot,
        m_ElevatorFeedforward.calculate(meters));
  }

  /*
   * Retract the elevator
   */
  public void LowerElevator() {
    SetHeight(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Update the simulation of our elevator, set inputs
    m_ElevatorSim.setInput(m_simMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // update simulation (20ms)
    m_ElevatorSim.update(0.020);

    // Iterate PID loops
    m_simMotor.iterate(
        m_ElevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);

    // add load to battery
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_ElevatorSim.getCurrentDrawAmps()));

    // update encoder
    m_ElevatorEncoderSim.setPosition(m_ElevatorSim.getPositionMeters());
    m_ElevatorEncoderSim.setVelocity(m_ElevatorSim.getVelocityMetersPerSecond());
  }
}
