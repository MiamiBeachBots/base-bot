// Copyright (c) Jack Nelson, Max Aitel & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** The claw command that uses the arm subsystem. */
public class ClawCommand extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private boolean openClaw = true;

  /**
   * Creates a new ClawCommand command.
   *
   * @param a_subsystem The arm subsystem used by this command.
   */
  public ClawCommand(ArmSubsystem a_subsystem) {
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
    if (this.openClaw) {
      m_armSubsystem.clawOpen();
    } else {
      m_armSubsystem.clawClose();
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
