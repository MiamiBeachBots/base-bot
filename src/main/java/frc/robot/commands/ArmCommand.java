package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/** An Arm command that uses the Arm subsystem. */
public class ArmCommand extends Command {
  private final ArmSubsystem m_ArmSubsystem;

  /**
   * Create a new ArmCommand.
   *
   * @param a_Subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem a_Subsystem) {
    m_ArmSubsystem = a_Subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a_Subsystem);
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
