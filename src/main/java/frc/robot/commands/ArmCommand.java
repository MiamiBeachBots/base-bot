// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

/** The default drive command that uses the drive subsystem. */
public class ArmCommand extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private final DoubleSupplier m_joy_x; // this gives us the x axis for current controller
  private final DoubleSupplier m_joy_y; // this gives us the y axis for current controller

  /**
   * Creates a new DefaultDrive command.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param joystick_x_func A function that returns the value of the x axis / extension axis for
   *     said joystick.
   * @param joystick_y_func A function that returns the value of the Y axis / height axis for said
   *     joystick.
   */
  public ArmCommand(
      ArmSubsystem a_subsystem, DoubleSupplier joystick_x_func, DoubleSupplier joystick_y_func) {
    m_armSubsystem = a_subsystem;
    m_joy_x = joystick_x_func;
    m_joy_y = joystick_y_func;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public void up() {
    m_armSubsystem.armElevator(0.0, 0.0);
  }

  public void down() {
    m_armSubsystem.armExtension(0.0, 0.0);
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
