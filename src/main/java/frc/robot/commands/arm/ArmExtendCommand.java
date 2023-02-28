// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import java.util.function.DoubleSupplier;

/** ArmExtend command that uses the extension subsystem. */
public class ArmExtendCommand extends CommandBase {
  private final ExtensionSubsystem m_extensionSubsystem;
  private final DoubleSupplier m_joy_y; // this gives us the y axis for current controller

  /**
   * Creates a new ArmExtend command.
   *
   * @param ext_subsystem The extension subsystem used by this command.
   * @param joystick_y_func A function that returns the value of the Y axis / height axis for said
   *     joystick.
   */
  public ArmExtendCommand(ExtensionSubsystem ext_subsystem, DoubleSupplier joystick_y_func) {
    m_extensionSubsystem = ext_subsystem;
    m_joy_y = joystick_y_func;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ext_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_extensionSubsystem.armExtend(m_joy_y.getAsDouble());
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