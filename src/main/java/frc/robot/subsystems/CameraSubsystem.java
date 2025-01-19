// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
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

  // Simulation Config
  // A vision system sim labelled as "main" in NetworkTables
  private final VisionSystemSim visionSim;
  // A 0.5 x 0.25 meter rectangular target
  private final TargetModel targetModel = new TargetModel(0.5, 0.25);
  // The pose of where the target is on the field.
  // Its rotation determines where "forward" or the target x-axis points.
  // Let's say this target is flat against the far wall center, facing the blue driver stations.
  private final Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  // The given target model at the given pose
  private final VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
  // setup cameras
  private final SimCameraProperties PoseCameraProp = new SimCameraProperties();
  private final SimCameraProperties TargetingCameraProp = new SimCameraProperties();
  private final PhotonCameraSim poseCamera1Sim;
  private final PhotonCameraSim poseCamera2Sim;
  private final PhotonCameraSim targetingCamera1Sim;

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

    // setup simulation for vision system
    visionSim = new VisionSystemSim("main");
    visionSim.addVisionTargets(visionTarget);
    visionSim.addAprilTags(aprilTagFieldLayout);

    // Set the properties of the camera
    TargetingCameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    PoseCameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));

    // Approximate detection noise with average and standard deviation error in pixels.
    TargetingCameraProp.setCalibError(0.25, 0.08);
    PoseCameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    TargetingCameraProp.setFPS(50);
    PoseCameraProp.setFPS(50);

    // The average and standard deviation in milliseconds of image data latency.
    TargetingCameraProp.setAvgLatencyMs(35);
    PoseCameraProp.setAvgLatencyMs(35);
    TargetingCameraProp.setLatencyStdDevMs(5);
    PoseCameraProp.setLatencyStdDevMs(5);

    // initialize the cameras
    poseCamera1Sim = new PhotonCameraSim(poseCamera1, PoseCameraProp);
    poseCamera2Sim = new PhotonCameraSim(poseCamera2, PoseCameraProp);
    targetingCamera1Sim = new PhotonCameraSim(targetingCamera1, TargetingCameraProp);

    // Set Camera locations and add them to the vision simulation
    visionSim.addCamera(poseCamera1Sim, Constants.PoseCamera1.location);
    visionSim.addCamera(poseCamera2Sim, Constants.PoseCamera2.location);
    visionSim.addCamera(targetingCamera1Sim, Constants.TargetingCamera1.location);
  }

  /**
   * Gets the last procesesd frame captured by camera
   *
   * @param camera Desired camera to get result from
   * @return Targets in the frame.
   */
  private Optional<PhotonPipelineResult> getPipelineResults(PhotonCamera camera) {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      SmartDashboard.putNumber("Front Camera Latency", result.getTimestampSeconds());
      if (result.hasTargets()) {
        // select last result with targets
        return Optional.of(result);
      }
    }
    return Optional.empty();
  }

  /**
   * Update estaimated robot pose based on given pipeline result.
   *
   * @param result Camera pipeline result
   * @param poseEstimator Pose estimator
   */
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
    // Update dashboard
    SmartDashboard.putBoolean("poseCamera1Connected", poseCamera1.isConnected());
    SmartDashboard.putBoolean("poseCamera2Connected", poseCamera2.isConnected());
    SmartDashboard.putBoolean("TargetingCamera1Connnected", targetingCamera1.isConnected());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Update with the simulated drivetrain pose. This should be called every loop in simulation.
    visionSim.update(m_driveSubsystem.getPose());
  }
}
