// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;

/** An example command that uses an example subsystem. */
public class AimCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonCamera m_camera;
  private final double SMALLDISTANCESPEED = 0.6;
  private final double LARGEDISTANCESPEED = 0.45;
  private final String CAMERANAME = "OV5647";
  private final NetworkTableEntry targetDetected;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimCommand(DriveSubsystem d_subsystem) {
    m_driveSubsystem = d_subsystem;

    // Change this to match the name of your camera
    m_camera = new PhotonCamera(CAMERANAME);

    // init networktables
    targetDetected = NetworkTableInstance.getDefault().getTable("").getEntry("targetDetected");

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d_subsystem);
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
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      // drivetrain too "fast"/not enough tourqe to be slow, very annoying
      if (artemCAMRESULT.getBestTarget().getPitch() > 10) {
        this.m_driveSubsystem.tankDrive(SMALLDISTANCESPEED, SMALLDISTANCESPEED);
      } else if (artemCAMRESULT.getBestTarget().getPitch() < -10) {
        this.m_driveSubsystem.tankDrive(-SMALLDISTANCESPEED, -SMALLDISTANCESPEED);
      } else if (artemCAMRESULT.getBestTarget().getPitch() < -2) {
        this.m_driveSubsystem.tankDrive(-LARGEDISTANCESPEED, -LARGEDISTANCESPEED);
      } else if (artemCAMRESULT.getBestTarget().getPitch() > 2) {
        this.m_driveSubsystem.tankDrive(LARGEDISTANCESPEED, LARGEDISTANCESPEED);
      }
    } else {
      targetDetected.setString("false");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
