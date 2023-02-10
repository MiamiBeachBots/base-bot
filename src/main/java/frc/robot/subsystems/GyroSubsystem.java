// Copyright (c) Jack Nelson & Miami Beach Bots
// Source: https://www.maxbotix.com/firstrobotics

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This Subsystem is what translates the output of the Gyro sensor to standard units. */
public class GyroSubsystem extends SubsystemBase {
  private final AHRS m_Gyro;

  /** Creates a new GyroSubsystem. */
  public GyroSubsystem() {
    m_Gyro = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void periodic() {}

  public void calibrate() {
    m_Gyro.calibrate();
  }

  // for balance correction
  public double getPitch() {
    return m_Gyro.getPitch(); // get pitch in degrees
  }

  // for PID control (turn by degrees)
  public double getAccumYaw() {
    return m_Gyro.getAngle(); // get angle in degrees
  }

  public double getYaw() {
    return m_Gyro.getYaw();
  }

  public void zeroYaw() {
    m_Gyro.zeroYaw();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
