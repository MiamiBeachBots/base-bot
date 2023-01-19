// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;

/** An example command that uses an example subsystem. */
public class AimCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonCamera m_camera;

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimCommand(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;

    // Change this to match the name of your camera
    m_camera = new PhotonCamera("Camera1");

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
    if (result.hasTargets()) {

      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      if (result.getBestTarget().getYaw() > 1) {
        System.out.println("222222");
        this.m_driveSubsystem.tankDrive(0.6, 0.6);
      } else if (result.getBestTarget().getYaw() < -1) {
        System.out.println("33333333333");
        this.m_driveSubsystem.tankDrive(-0.6, -0.6);
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
