// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AirSubsystem;

/** An example command that uses an example subsystem. */
public class AirCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AirSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_airSubsystem The subsystem used by this command.
   */
  public AirCommand(AirSubsystem m_airSubsystem) {
    m_subsystem = m_airSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
    m_subsystem.enableCompressor();
    m_subsystem.setSolenoid();
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {m_subsystem.toggleSolenoid();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
