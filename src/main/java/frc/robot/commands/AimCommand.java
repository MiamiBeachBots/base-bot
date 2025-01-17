// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** The Aim command that uses the camera + gyro to control the robot. */
public class AimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final CameraSubsystem m_cameraSubsystem;
  private final Transform3d robotOffset = new Transform3d();
  private final double toleranceMeters = 0.1;
  private Pose2d robotToTarget2d = new Pose2d();
  private Command resultingCommand;

  /**
   * Creates a new AimCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   */
  public AimCommand(DriveSubsystem d_subsystem, CameraSubsystem c_subsystem) {
    m_driveSubsystem = d_subsystem;
    m_cameraSubsystem = c_subsystem;

    // Change this to match the name of your camera

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem, c_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: update offset here
    // robotoffset = ....
    Optional<PhotonPipelineResult> CamResult = m_cameraSubsystem.frontCameraResult;
    // will not work if cam is defined incorrectly, but will not tell you
    CamResult.ifPresentOrElse(
        result -> {
          if (result.hasTargets()) {
            SmartDashboard.putBoolean("CameraTargetDetected", true);
            // find target we want, we can change later
            PhotonTrackedTarget target = result.getBestTarget();
            // we can change this to be a certain april tag later
            // https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html
            // get the transform from the camera to the target
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            // set offset of transform
            Transform3d targetOffset = cameraToTarget.plus(robotOffset);
            // get the pose of the robot
            Pose3d robotPose = new Pose3d(m_driveSubsystem.getPose());
            // add the offset to the robot pose
            Pose3d robotToTarget = robotPose.plus(targetOffset);
            // convert to a pose2d for the drive subsystem
            Pose2d newTargetPose = robotToTarget.toPose2d();
            // check if new pose within tolerance
            if (robotToTarget2d.getTranslation().getDistance(newTargetPose.getTranslation())
                > toleranceMeters) {
              // update the pose
              robotToTarget2d = newTargetPose;
              // update the drive subsystem
              resultingCommand = m_driveSubsystem.GenerateOnTheFlyCommand(robotToTarget2d);
              resultingCommand.initialize();
            }
          }
        },
        () -> {
          SmartDashboard.putBoolean("CameraTargetDetected", false);
          SmartDashboard.putNumber("CameraTargetPitch", 0.0);
        });
    if (resultingCommand != null) {
      resultingCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    resultingCommand.end(interrupted);
    m_driveSubsystem.stop(); // end execution of on board PID.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
