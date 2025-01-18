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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Motor
  private final SparkMax m_Motor;
  // Simulated Motor
  private final DCMotor m_Gearbox;
  private final SparkMaxSim m_MotorSim;

  // Motor Configs
  private final SparkMaxConfig m_MotorConfig = new SparkMaxConfig();
  // Declare PID
  private final SparkClosedLoopController m_ElevatorMainPIDController;
  // Declare Encoder
  private RelativeEncoder m_ElevatorEncoder;
  // Simulated Encoder
  private final SparkRelativeEncoderSim m_ElevatorEncoderSim;
  // TODO: Update to accurate values
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
  private final double kS = 0.2063; // Static Friction (Volts)
  private final double kG = 1.5611; // Inertia (Volts)
  private final double kV = 0.1396; // Mass (Volts*Seconds / Meter)
  private final double kA = 0.0; // Acceleration (Volts * Seconds^2 / Meter)

  ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  // current limit
  private final int k_CurrentLimit = 80;

  public ElevatorSubsystem() {
    // Create elevator motor
    m_Motor = new SparkMax(CANConstants.MOTORELEVATORID, SparkMax.MotorType.kBrushless);
    // Create Simulated Motors
    m_Gearbox = DCMotor.getNEO(1);
    m_MotorSim = new SparkMaxSim(m_Motor, m_Gearbox);

    // Set idle mode to coast
    m_MotorConfig.idleMode(IdleMode.kBrake);
    // Set current limit
    m_MotorConfig.smartCurrentLimit(k_CurrentLimit);

    // Connect to built in PID controller
    m_ElevatorMainPIDController = m_Motor.getClosedLoopController();

    // Allow us to read the encoder
    m_ElevatorEncoder = m_Motor.getEncoder();
    // Simulated encoder
    m_ElevatorEncoderSim = m_MotorSim.getRelativeEncoderSim();

    m_MotorConfig.encoder.positionConversionFactor(kPositionConversionRatio);
    m_MotorConfig.encoder.velocityConversionFactor(kVelocityConversionRatio);

    // PID coefficients
    kP = 0.00013373;
    kI = 0;
    kD = 0;
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    // set PID coefficients
    m_MotorConfig.closedLoop.pid(kP, kI, kD, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_MotorConfig.closedLoop.iZone(kIz, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_MotorConfig.closedLoop.outputRange(
        kMinOutput, kMaxOutput, DriveConstants.kDrivetrainVelocityPIDSlot);
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
   * Move elevator at a given speed (M/S)
   */
  public void MoveElevator(double speed) {
    m_ElevatorMainPIDController.setReference(
        speed,
        SparkBase.ControlType.kVelocity,
        DriveConstants.kDrivetrainVelocityPIDSlot,
        m_ElevatorFeedforward.calculate(speed));
  }

  public void MoveAtFull() {
    MoveElevator(1);
  }

  /*
   * Stop the elevator
   */
  public void StopElevator() {
    MoveElevator(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
