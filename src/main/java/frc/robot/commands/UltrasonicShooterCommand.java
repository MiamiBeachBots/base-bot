// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterState;
import frc.robot.subsystems.UltrasonicSubsystem;

/** An example command that uses an example subsystem. */
public class UltrasonicShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final UltrasonicSubsystem m_ultrasonicSubsystem;

  private final ShooterState m_shooterState;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public UltrasonicShooterCommand(UltrasonicSubsystem u_subsystem, ShooterState shooterState) {
    m_ultrasonicSubsystem = u_subsystem;
    m_shooterState = shooterState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ultrasonicSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_ultrasonicSubsystem.DistanceCM();
    if (distance <= 30) {
      m_shooterState.setLoaded();
    } else {
      m_shooterState.setFired();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
