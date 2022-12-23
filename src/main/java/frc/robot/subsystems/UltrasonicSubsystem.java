// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Source: https://www.maxbotix.com/firstrobotics

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSubsystem extends SubsystemBase {
  private final AnalogInput m_rawUltrasonic;
  private double voltageScaleFactor;

  /** Creates a new UltrasonicSubsystem. */
  public UltrasonicSubsystem(int PortNumber) {
    m_rawUltrasonic = new AnalogInput(PortNumber);
    voltageScaleFactor = 5 / RobotController.getVoltage5V();
  }

  public double DistanceIn() {
    // Gets distance from sensor and returns in inches
    return m_rawUltrasonic.getValue() * voltageScaleFactor * 0.0492;
  }

  public double DistanceCM() {
    // Gets distance from sensor and returns in centimeters
    return m_rawUltrasonic.getValue() * voltageScaleFactor * 0.125;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate what percentage of 5 Volts we are actually at
    voltageScaleFactor = 5 / RobotController.getVoltage5V();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
