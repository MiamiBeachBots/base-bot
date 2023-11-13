// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {

  final AHRS m_Gyro;
  public static Rotation2d Rotation2d;
  /** Creates a new ExampleSubsystem. */
  public GyroSubsystem() {
    // Init gyro
    m_Gyro = new AHRS(SPI.Port.kMXP);
    Rotation2d = this.getRotation2d();
    resetGyro();
  }

  public void calibrate() {
    m_Gyro.calibrate();
  }

  public Rotation2d getRotation2d() {
    return m_Gyro.getRotation2d();
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

  public void resetGyro() {
    m_Gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Gyro Pitch", getPitch());
    SmartDashboard.putNumber("Current Gyro Yaw", getYaw());
    Rotation2d = this.getRotation2d();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
