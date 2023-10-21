// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/** The default drive command that uses the drive subsystem. */
public class AutonomousCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final Timer m_timer = new Timer();
  private final double SPEED = 0.6;
  private final double TIME = 3.0;

  /**
   * Creates a new DefaultDrive command.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param xbox_left_y A function that returns the value of the left y axis for the joystick.
   * @param xbox_right_y A function that returns the value of the right Y axis for the joystick.
   */
  public AutonomousCommand(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // we include a limit on the drivers speed for safety.
    // Additonally the axis's on the
    this.m_driveSubsystem.tankDrive(Constants.MAX_SPEED * SPEED, Constants.MAX_SPEED * SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop(); // We might not want this
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(TIME);
  }
}
