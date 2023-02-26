// Copyright (c) Max Aitel, Jack Nelson & Miami Beach Bots

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // elevator motor
  private final CANSparkMax m_elevatorMotor;
  // built in pid controller
  private final SparkMaxPIDController m_elevatorPidController;

  public ElevatorSubsystem() {
    // arm up/y motor
    m_elevatorMotor = new CANSparkMax(CANConstants.ARMELEVATORMOTORID, MotorType.kBrushless);
    m_elevatorPidController = m_elevatorMotor.getPIDController();
    // configure elevator
  }

  public void armElevate(boolean direction) {
    // arm up down
    System.out.println("Arm is Elevating");
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
