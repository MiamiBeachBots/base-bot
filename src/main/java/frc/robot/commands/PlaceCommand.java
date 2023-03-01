// Copyright (c) Jack Nelson, Max Aitel & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ClawSubsystem;
import frc.robot.subsystems.arm.ElevatorSubsystem;

/** The claw command that uses the claw subsystem. */
public class PlaceCommand extends CommandBase {
  private final ClawSubsystem m_clawSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  /**
   * Creates a new AutoCommand command.
   *
   * @param a_subsystem The claw subsystem used by this command.
   * @param e_subsystem The Elevator subsystem.
   */
  public PlaceCommand(ClawSubsystem a_subsystem, ElevatorSubsystem e_subsystem) {
    m_clawSubsystem = a_subsystem;
    m_elevatorSubsystem = e_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a_subsystem);
    addRequirements(e_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // random number
    while (!m_elevatorSubsystem.moveArm(5)) {
      continue;
    }
    m_clawSubsystem.clawOpen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
