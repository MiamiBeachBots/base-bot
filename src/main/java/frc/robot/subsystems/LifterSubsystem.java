// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LifterSubsystem extends SubsystemBase {
  private final SparkMax m_motor; // Motor
  private final SparkMaxConfig m_motorConfig = new SparkMaxConfig(); // Motor Configuration
  // Simulated motor
  private final DCMotor m_gearbox;
  private final SparkMaxSim m_motorSim;
  private double kCurrentSpeed = Constants.LIFTER_SPEED;
  private int motorID;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem(int motor_ID) {
    motorID = motor_ID;
    // Create motor
    m_motor = new SparkMax(motorID, SparkMax.MotorType.kBrushless);
    // Create simulated motor
    m_gearbox = DCMotor.getNEO(1);
    m_motorSim = new SparkMaxSim(m_motor, m_gearbox);
    m_motorConfig.idleMode(IdleMode.kBrake);
    m_motorConfig.inverted(false);
    m_motorConfig.smartCurrentLimit(50);

    m_motor.configure(
        m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
