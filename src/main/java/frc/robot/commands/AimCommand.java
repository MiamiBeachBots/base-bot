// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/** An example command that uses an example subsystem. */
public class AimCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonCamera m_camera;
  private double FirstLevelSpeed = 0.6;
  private double SecondLevelSpeed = 0.45;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimCommand(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;

    // Change this to match the name of your camera
    m_camera = new PhotonCamera("OV5647");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = m_camera.getLatestResult();
    // will not work if cam is defined incorrectly, but will not tell you
    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      // drivetrain too "fast"/not enough tourqe to be slow, very annoying
      if (result.getBestTarget().getPitch() > 10) {
        this.m_driveSubsystem.tankDrive(FirstLevelSpeed, FirstLevelSpeed);
      } else if (result.getBestTarget().getPitch() < -10) {
        this.m_driveSubsystem.tankDrive(-FirstLevelSpeed, -FirstLevelSpeed);
      } else if (result.getBestTarget().getPitch() < -2) {
        this.m_driveSubsystem.tankDrive(-SecondLevelSpeed, -SecondLevelSpeed);
      } else if (result.getBestTarget().getPitch() > 2) {
        this.m_driveSubsystem.tankDrive(SecondLevelSpeed, SecondLevelSpeed);
      }
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
