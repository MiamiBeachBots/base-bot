// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShooterState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.HelperFunctions;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class ArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmSubsystem;

  private final ShooterState m_shooterState;
  private final DoubleSupplier m_yAxis;
  private final double kMaxRadiansPerInput = Units.degreesToRadians(5);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem a_subsystem, ShooterState shooterState, DoubleSupplier yAxis) {
    m_ArmSubsystem = a_subsystem;
    m_shooterState = shooterState;
    m_yAxis = yAxis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!HelperFunctions.inDeadzone(m_yAxis.getAsDouble(), Constants.CONTROLLERDEADZONE)) {
      m_ArmSubsystem.MoveArmRelative(m_yAxis.getAsDouble() * kMaxRadiansPerInput);

    } else if (m_shooterState.isLoaded & !m_shooterState.isLowered) {
      m_ArmSubsystem.lowerArm();
      m_shooterState.setLowered();
    } else {
      m_ArmSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
