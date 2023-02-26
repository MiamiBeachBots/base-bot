// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ElevatorSubsystem;

/** The Arm Up Command that uses the elevator subsystem */
public class ArmUpCommand extends CommandBase {
  private final ElevatorSubsystem m_elevatorSubsystem;

  /**
   * Creates a new Arm Up command.
   *
   * @param e_subsystem The elevator subsystem used by this command.
   */
  public ArmUpCommand(ElevatorSubsystem e_subsystem) {
    m_elevatorSubsystem = e_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.armElevate(true);
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
