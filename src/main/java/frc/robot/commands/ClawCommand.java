// Copyright (c) Jack Nelson, Max Aitel & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

/** The claw command that uses the claw subsystem. */
public class ClawCommand extends CommandBase {
  private final ClawSubsystem m_clawSubsystem;
  private boolean openClaw = true;

  /**
   * Creates a new ClawCommand command.
   *
   * @param a_subsystem The claw subsystem used by this command.
   */
  public ClawCommand(ClawSubsystem a_subsystem) {
    m_clawSubsystem = a_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.openClaw) {
      m_clawSubsystem.clawOpen();
    } else {
      m_clawSubsystem.clawClose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.openClaw = !this.openClaw; // change claw action on end.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
