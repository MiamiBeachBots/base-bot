// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class AutoCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private Timer robotTimer = new Timer();
  private boolean timer_complete = false;

  /**
   * Creates a new ShooterCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoCommand(
      DriveSubsystem d_subsystem, ShooterSubsystem s_subsystem, ElevatorSubsystem e_subsystem) {
    m_driveSubsystem = d_subsystem;
    m_shooterSubsystem = s_subsystem;
    m_elevatorSubsystem = e_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem, s_subsystem, e_subsystem);
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
      // m_robotContainer.doLift(1);
      // m_robotContainer.doElevatorLift(-1);
      m_shooterSubsystem.shoot(0.60);
    } else if (robotTimer.get() >= 4.5 && robotTimer.get() < 6) {
      m_elevatorSubsystem.lift(-1);
      m_shooterSubsystem.shoot(0.60);
    } else if (robotTimer.get() >= 6 && robotTimer.get() < 10) {
      backward();
      // m_robotContainer.doLift(0);
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
    stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer_complete;
  }
}
