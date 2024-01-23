// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

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
    PhotonPipelineResult CamResult = m_cameraSubsystem.frontCameraResult;
    // will not work if cam is defined incorrectly, but will not tell you
    if (CamResult.hasTargets()) {
      SmartDashboard.putBoolean("CameraTargetDetected", true);
      double angleGoal = m_driveSubsystem.getYaw() + CamResult.getBestTarget().getYaw();
      SmartDashboard.putNumber("CameraTargetPitch", angleGoal);
      double distanceFromTarget =
          PhotonUtils.calculateDistanceToTargetMeters(
                  m_cameraSubsystem.frontCameraHeightMeters,
                  m_cameraSubsystem.frontCameraTargetHeightMeters,
                  m_cameraSubsystem.frontCameraTargetPitchRadians,
                  Units.degreesToRadians(CamResult.getBestTarget().getPitch()))
              - m_cameraSubsystem.frontCameraGoalRangeMeters;
      // turn and move towards target.
      //m_driveSubsystem.driveAndTurn(m_driveSubsystem.getYaw(), angleGoal, distanceFromTarget);
      // we reset the angle everytime as the target could change / move.
      m_driveSubsystem.turnSetGoal(angleGoal);
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
    m_driveSubsystem.stop(); // end execution of on board PID.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
