// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** The Aim command that uses the camera + gyro to control the robot. */
public class AimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final CameraSubsystem m_cameraSubsystem;

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
            double angleGoal = m_driveSubsystem.getYaw() + target.getYaw();
            SmartDashboard.putNumber("CameraTargetPitch", angleGoal);
            double distanceFromTarget =
                PhotonUtils.calculateDistanceToTargetMeters(
                        m_cameraSubsystem.frontCameraHeightMeters,
                        m_cameraSubsystem.frontCameraTargetHeightMeters,
                        m_cameraSubsystem.frontCameraTargetPitchRadians,
                        Units.degreesToRadians(target.getPitch()))
                    - m_cameraSubsystem.frontCameraGoalRangeMeters;

            // turn and move towards target.
            // if within 0.5 pos or neg
            if (Math.abs(target.getYaw()) > 0.5) {
              m_driveSubsystem.turnSetGoal(angleGoal);
            } else if (Math.abs(distanceFromTarget) > 0.1) {
              m_driveSubsystem.driveToRelativePosition(distanceFromTarget);
            }
          }
        },
        () -> {
          SmartDashboard.putBoolean("CameraTargetDetected", false);
          SmartDashboard.putNumber("CameraTargetPitch", 0.0);
          m_driveSubsystem.tankDrive(0, 0);
        });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.turnResetPID(); // we clear the PID turn controller.
    m_driveSubsystem.stop(); // end execution of on board PID.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
