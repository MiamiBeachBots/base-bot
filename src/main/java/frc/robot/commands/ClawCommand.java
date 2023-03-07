// Copyright (c) Jack Nelson, Max Aitel & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ClawSubsystem;
import java.util.function.DoubleSupplier;

/** The claw command that uses the claw subsystem. */
public class ClawCommand extends CommandBase {
  private final ClawSubsystem m_clawSubsystem;
  private final DoubleSupplier m_joy_y;
  private boolean openClaw = true;

  /**
   * Creates a new ClawCommand command.
   *
   * @param a_subsystem The claw subsystem used by this command.
   * @param joystick_y_func A function that returns the value of the Y axis / height axis for said
   *     joystick.
   */
  public ClawCommand(ClawSubsystem a_subsystem, DoubleSupplier joystick_y_func) {
    m_joy_y = joystick_y_func;
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
    // if (this.openClaw) {
    //   m_clawSubsystem.clawOpen();
    // } else {
    //   m_clawSubsystem.clawClose();
    // }
    if (m_joy_y.getAsDouble() < 0.1 && m_joy_y.getAsDouble() > -0.1) {
      m_clawSubsystem.clawMotor(0.05);
    } else {
      m_clawSubsystem.clawMotor(m_joy_y.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_clawSubsystem.clawMotor(0.0); // change claw action on end.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
