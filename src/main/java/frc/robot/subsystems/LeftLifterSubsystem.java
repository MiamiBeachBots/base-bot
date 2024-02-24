// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

public class LeftLifterSubsystem extends SubsystemBase {
  private final CANSparkMax m_left; // Motor for Left
  private double kCurrentSpeed = Constants.LIFTERSPEED;

  /** Creates a new LifterSubsystem. */
  public LeftLifterSubsystem() {
    m_left = new CANSparkMax(CANConstants.MOTORLIFTERLEFTID, CANSparkMax.MotorType.kBrushless);
    m_left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_left.setInverted(false);
    m_left.burnFlash();
  }

  public void leftMove(double speed) {
    m_left.set(speed);
  }

  public void activateLeft() {
    leftMove(kCurrentSpeed);
  }

  public void changeDirection() {
    kCurrentSpeed = -kCurrentSpeed;
  }

  public void stopLeft() {
    m_left.set(0);
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
