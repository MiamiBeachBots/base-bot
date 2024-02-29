// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterState;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class LowerArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmSubsystem;

  private final ShooterState m_shooterState;

  /**
   * Creates a new ShootSpeakerCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LowerArmCommand(ArmSubsystem a_subsystem, ShooterState shooterState) {
    m_ArmSubsystem = a_subsystem;
    m_shooterState = shooterState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this bypasses the shooter state and just moves the arm to the speaker position, as the arm
    // subsystem gets paused during the auto shoot command
    m_shooterState.setMode(ShooterState.ShooterMode.DEFAULT);
    m_ArmSubsystem.lowerArm();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the arm is stopped, at goal, and the offset is set, then the command is finished, aka at
    // correct angle
    if (m_ArmSubsystem.ArmStopped() && m_ArmSubsystem.getFrontLimit()) {
      return true;
    } else {
      return false;
    }
  }
}
