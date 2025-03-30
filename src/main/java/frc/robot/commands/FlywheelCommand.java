// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterState;
import frc.robot.subsystems.FlywheelSubsystem;

/** A Shooter Command that uses an example subsystem. */
public class FlywheelCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FlywheelSubsystem m_shooterSubsystem;

  private final ShooterState m_shooterState;

  /**
   * Creates a new ShooterCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlywheelCommand(FlywheelSubsystem s_subsystem, ShooterState shooterState) {
    m_shooterSubsystem = s_subsystem;
    m_shooterState = shooterState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.SpinShooter(m_shooterState.getShooterSpeed());
    m_shooterState.startShooting();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterState.stopShooting();
    m_shooterSubsystem.StopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
