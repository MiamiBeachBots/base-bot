// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;


public class UltrasonicSubsystem extends SubsystemBase {
  private AnalogInput m_rawUltrasonic;
  private double voltageScaleFactor = 1;


  /** Creates a new UltrasonicSubsystem. */
  public UltrasonicSubsystem(int PortNumber) {
    m_rawUltrasonic = new AnalogInput(PortNumber);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
