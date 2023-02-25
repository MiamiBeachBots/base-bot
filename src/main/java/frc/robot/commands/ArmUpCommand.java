// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** The default drive command that uses the drive subsystem. */
public class ArmUpCommand extends CommandBase {
  private final ArmSubsystem m_armSubsystem;

  /**
   * Creates a new DefaultDrive command.
   *
   * @param d_subsystem The drive subsystem used by this command.
   */
  public ArmUpCommand(ArmSubsystem a_subsystem) {
    m_armSubsystem = a_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.armElevate(true);
  } // change speed when we have the pid or smth calibarted

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
