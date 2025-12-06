// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  SwerveDrive m_swerveDrive;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    // Initialzie swerve drive
    try {
      m_swerveDrive =
          new SwerveParser(Constants.SWERVE_JSON_DIRECTORY)
              .createSwerveDrive(Constants.SWERVE_MAX_SPEED);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    m_swerveDrive.setHeadingCorrection(
        false); // Heading correction should only be used while controlling the robot via angle.
    m_swerveDrive.setCosineCompensator(false); // Disables cosine compensator in simulation
    m_swerveDrive.setAngularVelocityCompensation(
        true, true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    m_swerveDrive.setModuleEncoderAutoSynchronize(
        false,
        1); // Resynchronizes absolute encoders and motor encoders periodically while not moving
  }

  /**
   * Primary drive method Calculates module states with a given {@link Translation2d} and a rotation
   * rate
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    m_swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs =
        SwerveMath.cubeTranslation(new Translation2d(xInput, yInput)); // Cube inputs for smoothing
    return m_swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.SWERVE_MAX_SPEED);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return m_swerveDrive.getPose();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return m_swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return m_swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return m_swerveDrive.swerveDriveConfiguration;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
