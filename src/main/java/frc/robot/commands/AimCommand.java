// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import org.photonvision.PhotonCamera;

/** The Aim command that uses the camera + gyro to control the robot. */
public class AimCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final GyroSubsystem m_gyroSubsystem;
  private final PhotonCamera m_camera;
  private final String CAMERANAME = "OV5647";
  private final NetworkTableEntry targetDetected;
  /**
   * Creates a new AimCommand.
   *
   * @param d_subsystem The drive subsystem used by this command.
   * @param g_subsystem The gyro subsystem used by this command.
   */
  public AimCommand(DriveSubsystem d_subsystem, GyroSubsystem g_subsystem) {
    m_driveSubsystem = d_subsystem;
    m_gyroSubsystem = g_subsystem;

    // Change this to match the name of your camera
    m_camera = new PhotonCamera(CAMERANAME);

    // init networktables
    targetDetected = NetworkTableInstance.getDefault().getTable("").getEntry("targetDetected");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem, g_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var artemCAMRESULT = m_camera.getLatestResult();
    // will not work if cam is defined incorrectly, but will not tell you
    if (artemCAMRESULT.hasTargets()) {
      targetDetected.setString("true");
      // use gyro PID with angle, very easy
      m_driveSubsystem.turnToAngle(
          m_gyroSubsystem.getYaw(), artemCAMRESULT.getBestTarget().getPitch());
      // we reset the angle everytime as the target could change / move.
      m_driveSubsystem.turnResetPID();
    } else {
      targetDetected.setString("false");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.turnResetPID(); // we make sure to clear the PID angle
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
