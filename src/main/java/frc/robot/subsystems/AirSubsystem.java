// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AirSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public AirSubsystem() {}
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  

  private final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  // we set the value of the solenoid to reverse, this lets us toggle
  public void setSolenoid() { // should pretty much only be used in init
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void toggleSolenoid() {
    m_doubleSolenoid.toggle();
  }

  public void enableCompressor(){
    m_compressor.enableDigital();
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
