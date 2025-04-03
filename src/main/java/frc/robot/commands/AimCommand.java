// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShooterState;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** The Aim command that uses the camera + gyro to control the robot. */
public class AimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final CameraSubsystem m_cameraSubsystem;
  private final ShooterState m_shooterState;
  private final Transform3d camOffset;
  private final Transform3d targetingOffset;
  private Command resultingCommand;
  private static final double kBallDiameter = Units.inchesToMeters(16.25);
  private static final double kpixelWidthAtSampleDistance = 0; // TODO
  private static final double kBallSampleDistance = 1; // Meters TODO

  /**
   * Creates a new AimCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   */
  public AimCommand(
      DriveSubsystem d_subsystem, CameraSubsystem c_subsystem, ShooterState shooterState) {
    m_driveSubsystem = d_subsystem;
    m_cameraSubsystem = c_subsystem;
    m_shooterState = shooterState;

    // Change this to match the name of your camera

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem, c_subsystem);

    // The first offset takes the camera location and converts to center of robot
    camOffset = Constants.TargetingCamera1.location;
    // this offset takes the center of robot and tells it to move back so that we dont just run over
    // the ball
    targetingOffset = camOffset.plus(Constants.AlgaeCamOffset.location);
  }

  // Called when the command is initially schedule
  @Override
  public void initialize() {
    resultingCommand = null;
  }

  /**
   * Takes pipeline result from camera, follows path based on those results.
   *
   * @param result Camera pipeline result
   */
  private void processResult(PhotonPipelineResult result) {
    SmartDashboard.putBoolean("CameraTargetDetected", true);
    // find target we want, we can change later
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d cameraToTarget = distanceToTarget(target);

    Logger.recordOutput("AimCamToTargetTransform", cameraToTarget);

    // Now take target transform and apply to target coords
    // This essentially makes them relative to robot pose, then relative to intake
    Transform3d targetOffset = cameraToTarget.plus(targetingOffset);

    Logger.recordOutput("AimTargetRelRobotPose", targetOffset);

    // get the pose of the robot
    Pose3d robotPose = new Pose3d(m_driveSubsystem.getPose());

    // add the offset to the robot pose (now relative to field)
    Pose3d robotToTarget = robotPose.plus(targetOffset);

    // Log ball pose
    Logger.recordOutput("AimNavRelPose", robotToTarget);

    // convert to a pose2d for the drive subsystem
    Pose2d newTargetPose = robotToTarget.toPose2d();

    Logger.recordOutput("AimNav2dPose", newTargetPose);

    // calculate rotation
    Rotation2d newRotation = new Rotation2d(newTargetPose.getRotation().getDegrees());

    // check if new pose within tolerance
    // Create list of target poses
    // One at halfway to target, one at the target
    List<Pose2d> targetPoses = new ArrayList<Pose2d>();
    targetPoses.add(
        new Pose2d(
            robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), newRotation));
    targetPoses.add(
        new Pose2d(
            newTargetPose.getTranslation().getX(),
            newTargetPose.getTranslation().getY(),
            newRotation));

    // update the drive subsystem
    if (m_shooterState.isElevatorLowered) {
      m_driveSubsystem.setReducedSpeed(false);
    } else {
      m_driveSubsystem.setReducedSpeed(true);
    }
    resultingCommand = m_driveSubsystem.GenerateOnTheFlyCommand(targetPoses);
    resultingCommand.initialize();
  }

  // Finds the distance from the camera to a target
  private Transform3d distanceToTarget(PhotonTrackedTarget target) {
    double sampleDistance = kBallSampleDistance;
    List<TargetCorner> targetCorners = target.getDetectedCorners();
    double minX = Double.MAX_VALUE;
    double maxX = -Double.MAX_VALUE;

    // Loop through each corner and update minX and maxX
    for (TargetCorner corner : targetCorners) {
      if (corner.x < minX) {
        minX = corner.x;
      }
      if (corner.x > maxX) {
        maxX = corner.x;
      }
    }
    double currentPixelWidth = maxX - minX;
    Logger.recordOutput("AimCurPixWidth", currentPixelWidth);
    double ratio = kpixelWidthAtSampleDistance * sampleDistance / kBallDiameter; // shouldnt change
    double distance = kBallDiameter * ratio / currentPixelWidth;
    double yaw = Units.degreesToRadians(target.getYaw()); // rel x axis
    double distance_x = distance * Math.cos(yaw);
    double distance_y = distance * Math.sin(yaw);
    return new Transform3d(new Translation3d(distance_x, distance_y, 0), new Rotation3d(0, 0, yaw));
  }

  // Called every time the cheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (resultingCommand != null) {
      resultingCommand.execute();
    } else {
      Optional<PhotonPipelineResult> CamResult = m_cameraSubsystem.targetingCamera1Result;
      // will not work if cam is defined incorrectly, but will not tell you
      CamResult.ifPresentOrElse(
          result -> {
            if (result.hasTargets()) {
              processResult(result);
            }
          },
          () -> {
            SmartDashboard.putBoolean("CameraTargetDetected", false);
            SmartDashboard.putNumber("CameraTargetPitch", 0.0);
          });
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (resultingCommand != null) {
      resultingCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
