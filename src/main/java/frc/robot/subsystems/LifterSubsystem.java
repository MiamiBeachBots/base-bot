// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LifterSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor; // Motor for
  private double kCurrentSpeed = Constants.LIFTERSPEED;
  private int motorID;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem(int motor_ID) {
    motorID = motor_ID;
    m_motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
    m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(60);
    m_motor.burnFlash();
  }

  public void move(double speed) {
    m_motor.set(speed);
  }

  public void activate() {
    move(kCurrentSpeed);
  }

  public void changeDirection() {
    kCurrentSpeed = -kCurrentSpeed;
  }

  public void stop() {
    m_motor.set(0);
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
