// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class DriftCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_left_y; // this gives us the left y axis for current controller
  private final DoubleSupplier m_right_y; // this gives us the right y axis for current controller
  /**
   * Creates a new StraightCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param xbox_left_y A function that returns the value of the left y axis for the joystick.
   * @param xbox_right_y A function that returns the value of the right Y axis for the joystick.
   */
  public DriftCommand(
      DriveSubsystem d_subsystem, DoubleSupplier xbox_left_y, DoubleSupplier xbox_right_y) {
    m_driveSubsystem = d_subsystem;
    m_left_y = xbox_left_y;
    m_right_y = xbox_right_y;

    // Change this to match the name of your camera
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting 'StraightCommand.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.tankDrive(
        Constants.MAX_SPEED_SPRINT * m_left_y.getAsDouble(),
        Constants.MAX_SPEED_SPRINT * m_right_y.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
