package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurntableSubsystem;

/** An Turntable command that uses the Turntable subsystem. */
public class TurntableCommand extends Command {
  private final TurntableSubsystem m_TurntableSubsystem;

  /**
   * Create a new TurntableCommand.
   *
   * @param t_Subsystem The subsystem used by this command.
   */
  public TurntableCommand(TurntableSubsystem t_Subsystem) {
    m_TurntableSubsystem = t_Subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(t_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
