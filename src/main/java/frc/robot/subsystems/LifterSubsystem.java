// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class LifterSubsystem extends SubsystemBase {
  private final CANSparkMax m_left; // Motor for Left
  private final CANSparkMax m_right; // Motor for right
  private final double speed = 0.5;

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem() {
    m_left = new CANSparkMax(CANConstants.MOTORLIFTERLEFTID, CANSparkMax.MotorType.kBrushless);
    m_right = new CANSparkMax(CANConstants.MOTORLIFTERRIGHTID, CANSparkMax.MotorType.kBrushless);
  }

  public void leftUp() {
    m_left.set(speed);
  }

  public void leftDown() {
    m_left.set(-speed);
  }

  public void rightUp() {
    m_right.set(speed);
  }

  public void rightDown() {
    m_right.set(-speed);
  }

  public void stop() {
    m_left.set(0);
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
