// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/** The default drive command that uses the drive subsystem. */
public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_joy_x; // this gives us the x axis for current controller
  private final DoubleSupplier m_joy_y; // this gives us the y axis for current controller

  /**
   * Creates a new DefaultDrive command.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param joystick_throttle_func A function that returns the value of the x axis / throttle axis
   *     for the joystick.
   * @param joystick_y_func A function that returns the value of the Y axis for the joystick.
   */
  public DefaultDrive(
      DriveSubsystem d_subsystem,
      DoubleSupplier joystick_throttle_func,
      DoubleSupplier joystick_y_func) {
    m_driveSubsystem = d_subsystem;
    m_joy_x = joystick_throttle_func;
    m_joy_y = joystick_y_func;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // we include a limit on the drivers speed for safety.
    // Additonally the axis's on the
    this.m_driveSubsystem.tankDrive(
        Constants.MAX_SPEED * -m_joy_x.getAsDouble(), Constants.MAX_SPEED * -m_joy_y.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop(); // We might not want this
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
