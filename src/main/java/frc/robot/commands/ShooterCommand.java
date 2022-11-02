// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends CommandBase {
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DoubleSupplier m_joy_y; // this gives us the y axis for current big joystick

  /**
   * Creates a new ShooterCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(
      ShooterSubsystem s_subsystem, ElevatorSubsystem e_subsystem, DoubleSupplier joystick_y_func) {
    m_shooterSubsystem = s_subsystem;
    m_elevatorSubsystem = e_subsystem;
    m_joy_y = joystick_y_func;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_subsystem, e_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.lift(1);
    m_shooterSubsystem.shoot(0.60);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.lift(m_joy_y.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.lift(0);
    m_shooterSubsystem.shoot(0);
    m_elevatorSubsystem.lift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
