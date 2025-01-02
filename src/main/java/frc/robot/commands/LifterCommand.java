// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LifterSubsystem;

/** Lifter Command using the Lifter Subsystem. */
public class LifterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LifterSubsystem m_lifterSubsystem;

  /**
   * Creates a new LifterDownCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LifterCommand(LifterSubsystem l_subsystem) {
    m_lifterSubsystem = l_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(l_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lifterSubsystem.activate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lifterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
