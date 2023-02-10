// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/** A Balancing command that uses the gyro subsystem. */
public class BalanceCommand extends CommandBase {
  private final GyroSubsystem m_gyroSubsystem;
  private final DriveSubsystem m_driveSubsystem;

  /**
   * Creates a new BalanceCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param g_subsystem The gyro subsystem used by this command.
   */
  public BalanceCommand(DriveSubsystem d_subsystem, GyroSubsystem g_subsystem) {
    m_driveSubsystem = d_subsystem;
    m_gyroSubsystem = g_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem, m_gyroSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting balancing.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.balanceCorrection(m_gyroSubsystem.getPitch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending balancing.");
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
