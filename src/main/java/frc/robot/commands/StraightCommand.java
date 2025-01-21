// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class StraightCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private Command resultingCommand;

  /**
   * Creates a new StraightCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   */
  public StraightCommand(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;

    // Change this to match the name of your camera
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting 'StraightCommand.");
    resultingCommand = m_driveSubsystem.driveStraight();
    resultingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // generate new command every 100ms
    resultingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    resultingCommand.end(interrupted);
    System.out.println("Ending 'StraightCommand.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
