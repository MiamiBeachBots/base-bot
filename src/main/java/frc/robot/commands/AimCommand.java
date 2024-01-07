// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/** The Aim command that uses the camera + gyro to control the robot. */
public class AimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonCamera m_camera;
  private final String CAMERANAME = "OV5647";
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(0.5);

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  /**
   * Creates a new AimCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   */
  public AimCommand(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;

    // Change this to match the name of your camera
    m_camera = new PhotonCamera(CAMERANAME);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult CamResult = m_camera.getLatestResult();
    // will not work if cam is defined incorrectly, but will not tell you
    if (CamResult.hasTargets()) {
      SmartDashboard.putBoolean("CameraTargetDetected", true);
      double angleGoal = m_driveSubsystem.getYaw() + CamResult.getBestTarget().getYaw();
      SmartDashboard.putNumber("CameraTargetPitch", angleGoal);
      double distanceFromTarget =
          PhotonUtils.calculateDistanceToTargetMeters(
                  CAMERA_HEIGHT_METERS,
                  TARGET_HEIGHT_METERS,
                  CAMERA_PITCH_RADIANS,
                  Units.degreesToRadians(CamResult.getBestTarget().getPitch()))
              - GOAL_RANGE_METERS;
      // turn and move towards target.
      m_driveSubsystem.driveAndTurn(m_driveSubsystem.getYaw(), angleGoal, distanceFromTarget);
      // we reset the angle everytime as the target could change / move.
      m_driveSubsystem.turnSetGoal(angleGoal);
      m_driveSubsystem.distanceSetGoal(distanceFromTarget);
    } else {
      SmartDashboard.putBoolean("CameraTargetDetected", false);
      SmartDashboard.putNumber("CameraTargetPitch", 0.0);
      m_driveSubsystem.tankDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.turnResetPID(); // we clear the PID turn controller.
    m_driveSubsystem.distanceResetPID(); // we clear the distance PID contoller too.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
