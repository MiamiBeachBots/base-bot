// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final SparkPIDController m_armPIDController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kLoweredArmPosition;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // create the arm motor
    m_armMotor = new CANSparkMax(CANConstants.MOTORARMID, CANSparkMax.MotorType.kBrushless);
    m_armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // connect to built in PID controller
    m_armPIDController = m_armMotor.getPIDController();
    // allow us to read the encoder
    m_encoder = m_armMotor.getEncoder();
    // PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    kLoweredArmPosition = 0;

    // set PID coefficients
    m_armPIDController.setP(kP);
    m_armPIDController.setI(kI);
    m_armPIDController.setD(kD);
    m_armPIDController.setIZone(kIz);
    m_armPIDController.setFF(kFF);
    m_armPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /*
   * Move arm relative to current position
   */
  public void MoveArmRelative(double rotations) {
    rotations = rotations + m_encoder.getPosition();
    // update the PID controller with current encoder position
    m_armPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);
  }

  public void lowerArm() {
    // update the PID controller with current encoder position
    m_armPIDController.setReference(kLoweredArmPosition, CANSparkBase.ControlType.kPosition);
  }

  /*
   * Move arm to global position
   */
  public void MoveArmToPosition(double rotations) {
    // update the PID controller with current encoder position
    m_armPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);
  }

  /*
   * attempt to hold arm at current location
   */
  public void stop() {
    m_armPIDController.setReference(m_encoder.getPosition(), CANSparkBase.ControlType.kPosition);
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
