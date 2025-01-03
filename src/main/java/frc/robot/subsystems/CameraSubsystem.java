// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraSubsystem extends SubsystemBase {
  private final DriveSubsystem m_driveSubsystem;
  public final AprilTagFieldLayout aprilTagFieldLayout;
  private final String frontCameraName = "cam";
  private final PhotonCamera frontCamera;
  public PhotonPipelineResult frontCameraResult;
  // Physical location of camera relative to center
  private final double CameraLocationXMeters = Units.inchesToMeters(6);
  private final double CameraLocationYMeters = Units.inchesToMeters(9.3);
  private final double CameraLocationZMeters = Units.inchesToMeters(10.5);
  // angle of camera / orientation

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private final double CameraRollRadians = Units.degreesToRadians(90);
  private final double CameraPitchRadians = Units.degreesToRadians(0.0);
  private final double CameraYawRadians = Units.degreesToRadians(0);

  private final Transform3d frontCameraLocation =
      new Transform3d(
          new Translation3d(CameraLocationXMeters, CameraLocationYMeters, CameraLocationZMeters),
          new Rotation3d(CameraRollRadians, CameraPitchRadians, CameraYawRadians));

  private final PhotonPoseEstimator frontCameraPoseEstimator;
  // Constants such as camera and target height stored. Change per robot and goal!
  public final double frontCameraHeightMeters = Units.inchesToMeters(24);

  // Target Configruation for the front camera
  public final double frontCameraTargetHeightMeters = Units.feetToMeters(0.5);
  // Angle between horizontal and the camera.
  public final double frontCameraTargetPitchRadians = Units.degreesToRadians(0);
  // How far from the target we want to be
  public final double frontCameraGoalRangeMeters = Units.feetToMeters(3);

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    frontCamera = new PhotonCamera(frontCameraName);
    frontCameraPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCameraLocation);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return frontCameraPoseEstimator.update(frontCameraResult);
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose();
    if (pose.isPresent()) {
      SmartDashboard.putBoolean("CameraConnnected", true);
      m_driveSubsystem.updateVisionPose(
          pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
    } else {
      SmartDashboard.putBoolean("CameraConnnected", false);
    }
    frontCameraResult = frontCamera.getLatestResult();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Camera Latency", frontCameraResult.getTimestampSeconds());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
