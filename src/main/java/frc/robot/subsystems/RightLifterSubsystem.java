// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

public class RightLifterSubsystem extends SubsystemBase {
  private final CANSparkMax m_right; // Motor for right
  private double kCurrentSpeed = Constants.LIFTERSPEED;

  /** Creates a new LifterSubsystem. */
  public RightLifterSubsystem() {
    m_right = new CANSparkMax(CANConstants.MOTORLIFTERRIGHTID, CANSparkMax.MotorType.kBrushless);
    m_right.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_right.burnFlash();
  }

  public void rightMove(double speed) {
    m_right.set(speed);
  }

  public void activateRight() {
    rightMove(kCurrentSpeed);
  }

  public void changeDirection() {
    kCurrentSpeed = -kCurrentSpeed;
  }

  public void stopRight() {
    m_right.set(0);
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
