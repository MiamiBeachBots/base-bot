// Copyright (c) Jack Nelson & Miami Beach Bots
// Source: https://www.maxbotix.com/firstrobotics

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This Subsystem is what translates the output of the Gyro sensor to standard units. */
public class GyroSubsystem extends SubsystemBase {
  private final ADIS16448_IMU m_Gyro;

  /** Creates a new GyroSubsystem. */
  public GyroSubsystem() {
    m_Gyro = new ADIS16448_IMU();
    System.out.println(m_Gyro.getPort());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double error = m_Gyro.getRate();
    // System.out.println("Gyro:");
    // System.out.println(error);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
