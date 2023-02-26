// Copyright (c) Max Aitel, Jack Nelson & Miami Beach Bots

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // elevator motor
  private final CANSparkMax m_elevatorMotor;
  // built in pid controller
  private final SparkMaxPIDController m_elevatorPidController;
  // built in encoder
  private final RelativeEncoder m_elevatorEncoder;

  // Elevator PIDF constants
  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;
  private final double kIz = 0;
  private final double kFF = 0;
  // Other Speed Constants
  private final double kMaxOutput = 1;
  private final double kMinOutput = -1;
  // Rotation Constants
  private final float kmaxRotations = 0;
  private final double rotationsPerCall = 5;

  public ElevatorSubsystem() {
    // arm up/y motor
    m_elevatorMotor = new CANSparkMax(CANConstants.ARMELEVATORMOTORID, MotorType.kBrushless);
    m_elevatorPidController = m_elevatorMotor.getPIDController();
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    // configure elevator
    m_elevatorPidController.setP(kP);
    m_elevatorPidController.setI(kI);
    m_elevatorPidController.setD(kD);
    m_elevatorPidController.setOutputRange(kMinOutput, kMaxOutput);
    m_elevatorPidController.setIZone(kIz);
    m_elevatorPidController.setFF(kFF);
    m_elevatorMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_elevatorMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kmaxRotations);
    m_elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
  }

  public void armElevate(boolean direction) {
    System.out.println("Arm is Elevating");
    if (direction) {
      // move up
      moveArm(rotationsPerCall);
    } else {
      // move down
      moveArm(-rotationsPerCall);
    }
  }

  private void moveArm(double rotations) {
    double curPos = m_elevatorEncoder.getPosition();
    double newPos = curPos + rotations;
    if (newPos > kmaxRotations) {
      newPos = kmaxRotations;
    } else if (newPos < 0) {
      newPos = 0;
    }
    m_elevatorPidController.setReference(newPos, CANSparkMax.ControlType.kPosition);
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
