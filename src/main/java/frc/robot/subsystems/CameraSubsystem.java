// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraSubsystem extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;
  public final AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera poseCamera1;
  private final PhotonCamera poseCamera2;
  private final PhotonCamera targetingCamera1;

  public Optional<PhotonPipelineResult> poseCamera1Result;
  public Optional<PhotonPipelineResult> poseCamera2Result;
  public Optional<PhotonPipelineResult> targetingCamera1Result;

  private final PhotonPoseEstimator poseCamera1PoseEstimator;
  private final PhotonPoseEstimator poseCamera2PoseEstimator;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    poseCamera1 = new PhotonCamera(Constants.PoseCamera1.name);
    poseCamera2 = new PhotonCamera(Constants.PoseCamera2.name);
    targetingCamera1 = new PhotonCamera(Constants.TargetingCamera1.name);

    poseCamera1PoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.PoseCamera1.location);
    poseCamera2PoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.PoseCamera2.location);
    poseCamera1PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseCamera2PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  private Optional<PhotonPipelineResult> getPipelineResults(PhotonCamera camera) {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // select last result with targets
        SmartDashboard.putBoolean("CameraConnnected", true);
        SmartDashboard.putNumber("Front Camera Latency", result.getTimestampSeconds());
        return Optional.of(result);
      }
    }
    SmartDashboard.putBoolean("CameraConnnected", false);
    return Optional.empty();
  }

  private void updateGlobalPose(
      Optional<PhotonPipelineResult> result, PhotonPoseEstimator poseEstimator) {
    if (result.isPresent()) {
      Optional<EstimatedRobotPose> curPose = poseEstimator.update(result.get());
      if (curPose.isPresent()) {
        m_driveSubsystem.updateVisionPose(
            curPose.get().estimatedPose.toPose2d(), curPose.get().timestampSeconds);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // update the pipeline results
    poseCamera1Result = getPipelineResults(poseCamera1);
    poseCamera2Result = getPipelineResults(poseCamera2);
    targetingCamera1Result = getPipelineResults(targetingCamera1);
    // update the pose estimators
    updateGlobalPose(poseCamera1Result, poseCamera1PoseEstimator);
    updateGlobalPose(poseCamera2Result, poseCamera2PoseEstimator);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
