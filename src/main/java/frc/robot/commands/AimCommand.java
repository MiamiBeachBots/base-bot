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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShooterState;
import frc.robot.ShooterState.ShooterMode;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** The Aim command that uses the camera + gyro to control the robot. */
public class AimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final CameraSubsystem m_cameraSubsystem;
  private final ShooterState m_shooterState;
  private final Transform3d camOffset;
  private final Transform3d targetingOffset;
  private final double toleranceMeters = 0.1;
  private Pose2d lastRobotToTarget2d = new Pose2d();
  private Command resultingCommand;

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
    ShooterMode intakeMode = ShooterState.ShooterModes.INTAKE;
    camOffset = Constants.TargetingCamera1.modifiedTransform(intakeMode.height, intakeMode.angle);
    // this offset takes the center of robot and tells it to move back so that we dont just run over
    // the ball
    targetingOffset = camOffset.plus(Constants.AlgaeCamOffset.location);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastRobotToTarget2d = new Pose2d();
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

    // we can change this to be a certain april tag later
    // https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html
    // get the transform from the camera to the target
    double yaw = target.getYaw();
    double area = target.getArea();

    Logger.recordOutput("AimTargetYaw", yaw);
    Logger.recordOutput("AimTargetArea", area);


    // if area less then 10% do 1.5 meter otherwise do 0.5
    double targetDistance;
    if (area < 0.1) {
      targetDistance = 1.5;
    }
    else if (area > 0.7) {
      targetDistance = 0;
    }
    else {
      targetDistance = 0.5;
    }
    Transform3d cameraToTarget =
        new Transform3d(new Translation3d(0, targetDistance, 0), new Rotation3d(0, 0, yaw));

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
    // check if new pose within tolerance
    if (lastRobotToTarget2d.getTranslation().getDistance(newTargetPose.getTranslation())
        > toleranceMeters) {
      // Create list of target poses
      // One at halfway to target, one at the target
      List<Pose2d> targetPoses = new ArrayList<Pose2d>();
      targetPoses.add(
          new Pose2d(
              newTargetPose.getTranslation().getX() / 2,
              newTargetPose.getTranslation().getY() / 2,
              new Rotation2d(newTargetPose.getRotation().getDegrees())));
      targetPoses.add(
          new Pose2d(
              newTargetPose.getTranslation().getX(),
              newTargetPose.getTranslation().getY(),
              new Rotation2d(newTargetPose.getRotation().getDegrees())));

      // update the drive subsystem
      if (m_shooterState.isElevatorLowered) {
        m_driveSubsystem.setReducedSpeed(false);
      } else {
        m_driveSubsystem.setReducedSpeed(true);
      }
      resultingCommand = m_driveSubsystem.GenerateOnTheFlyCommand(targetPoses);
      resultingCommand.initialize();
      // update the pose
      lastRobotToTarget2d = newTargetPose;
    }
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
