// Copyright (c) Jack Nelson & Miami Beach Bots
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/** The autonomous mode command that uses the drive subsystem. */
public class AutoCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final GyroSubsystem m_gyroSubsystem;
  private Timer robotTimer = new Timer();
  private boolean timer_complete = false;

  /**
   * Creates a new AutoCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param g_subsystem The gyro subsystem used by this command.
   */
  public AutoCommand(DriveSubsystem d_subsystem, GyroSubsystem g_subsystem) {
    m_driveSubsystem = d_subsystem;
    m_gyroSubsystem = g_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem, g_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (robotTimer.get() <= 4) {
      stop();
      // also shoot here
    } else if (robotTimer.get() >= 4.5 && robotTimer.get() < 6) {
      m_driveSubsystem.turnToAngle(m_gyroSubsystem.getYaw(), 90);
    } else if (robotTimer.get() >= 6 && robotTimer.get() < 10) {
      m_driveSubsystem.resetPID();
      backward();
      m_driveSubsystem.turnToAngle(m_gyroSubsystem.getYaw(), -90);
    } else {
      timer_complete = true;
    }
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
    m_driveSubsystem.resetPID();
    stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer_complete;
  }
}
