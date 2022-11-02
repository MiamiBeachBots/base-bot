// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_joy_x; // this gives us the x axis for current controller
  private final DoubleSupplier m_joy_y; // this gives us the y axis for current controller

  /**
   * Creates a new ShooterCommand.
   *
   * @param subsystem The subsystem used by this command.
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
    this.m_driveSubsystem.tankDrive(
        Constants.MAX_SPEED * m_joy_x.getAsDouble(), -Constants.MAX_SPEED * m_joy_y.getAsDouble());
  }

  public void backward() {
    m_driveSubsystem.backward();
  }

  public void stop() {
    m_driveSubsystem.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop(); // We might not want this
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
