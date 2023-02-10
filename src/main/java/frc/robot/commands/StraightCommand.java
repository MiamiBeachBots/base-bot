// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class StraightCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final GyroSubsystem m_gyroSubsystem;
  private final DoubleSupplier m_joy_x;
  private final DoubleSupplier m_joy_y;
  /**
   * Creates a new StraightCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param g_subsystem The gyro subsystem used by this command.
   * @param joystick_throttle_func A function that returns the value of the x axis / throttle axis
   *     for the joystick.
   * @param joystick_y_func A function that returns the value of the Y axis for the joystick.
   */
  public StraightCommand(
      DriveSubsystem d_subsystem,
      GyroSubsystem g_subsystem,
      DoubleSupplier joystick_throttle_func,
      DoubleSupplier joystick_y_func) {
    m_driveSubsystem = d_subsystem;
    m_gyroSubsystem = g_subsystem;
    m_joy_x = joystick_throttle_func;
    m_joy_y = joystick_y_func;

    // Change this to match the name of your camera
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem, g_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting 'StraightCommand.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.driveStraight(
        m_gyroSubsystem.getYaw(),
        m_gyroSubsystem.getAccumYaw(),
        (m_joy_x.getAsDouble() + m_joy_y.getAsDouble()) / 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.resetPID(); // we make sure to clear the PID angle
    System.out.println("Ending 'StraightCommand.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
