package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterState;
import frc.robot.ShooterState.ShooterModes;
import frc.robot.subsystems.ArmSubsystem;

/** An Arm command that uses the Arm subsystem. */
public class ArmCommand extends Command {
  private final ArmSubsystem m_ArmSubsystem;
  private final ShooterState m_shooterState;

  /**
   * Create a new ArmCommand.
   *
   * @param a_Subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem a_Subsystem, ShooterState shooterState) {
    m_ArmSubsystem = a_Subsystem;
    m_shooterState = shooterState;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a_Subsystem);
  }

  // Update the states of the arm
  public void updateStates() {
    // Update lowered state of the arm
    // if the arm is at the goal and the arm is completely lowered, set lowered to true.
    m_shooterState.setArmResting(
        m_ArmSubsystem.atGoal() && m_shooterState.currentMode.angle == ShooterModes.DEFAULT.angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.SetAngle(m_shooterState.currentMode.angle);
    updateStates();
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
