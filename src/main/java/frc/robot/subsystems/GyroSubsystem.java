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

  public double getPitch() {
    return m_Gyro.getPitch(); // get pitch in degrees
  }

  public double getRoll() {
    return m_Gyro.getRoll(); // get roll in degrees
  }

  public double balanceCorrection() { // pitch radians
    double pitchAngleRadians = this.getPitch() * (Math.PI / 180.0);
    double xAxisRate = Math.sin(pitchAngleRadians) * -1;
    return xAxisRate; // return from sin;
  }

  public double getAngle() {
    return m_Gyro.getAngle(); // get angle in degrees
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
