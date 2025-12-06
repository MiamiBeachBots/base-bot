// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** An example command that uses an example subsystem. */
public class SwerveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_swerveSubsystem;

  private final DoubleSupplier m_vX, m_vY;
  private final DoubleSupplier m_headingHorizontal, m_headingVertical;
  private boolean m_initRotation = false;

  /**
   * Used to drive a swerve robot in full field-centric mode. vX and vY supply translation inputs,
   * where x is torwards/away from alliance wall and y is left/right. headingHorzontal and
   * headingVertical are the Cartesian coordinates from which the robot's angle will be derived—
   * they will be converted to a polar angle, which the robot will rotate to.
   *
   * @param s_subsystem The swerve drivebase subsystem.
   * @param vX DoubleSupplier that supplies the x-translation joystick input. Should be in the range
   *     -1 to 1 with deadband already accounted for. Positive X is away from the alliance wall.
   * @param vY DoubleSupplier that supplies the y-translation joystick input. Should be in the range
   *     -1 to 1 with deadband already accounted for. Positive Y is towards the left wall when
   *     looking through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's
   *     heading angle. In the robot coordinate system, this is along the same axis as vY. Should
   *     range from -1 to 1 with no deadband. Positive is towards the left wall when looking through
   *     the driver station glass.
   * @param headingVertical DoubleSupplier that supplies the vertical component of the robot's
   *     heading angle. In the robot coordinate system, this is along the same axis as vX. Should
   *     range from -1 to 1 with no deadband. Positive is away from the alliance wall.
   */
  public SwerveCommand(
      SwerveSubsystem s_subsystem,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier headingHorizontal,
      DoubleSupplier headingVertical) {
    m_swerveSubsystem = s_subsystem;
    m_vX = vX;
    m_vY = vY;
    m_headingHorizontal = headingHorizontal;
    m_headingVertical = headingVertical;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds desiredSpeeds =
        m_swerveSubsystem.getTargetSpeeds(
            m_vX.getAsDouble(),
            m_vY.getAsDouble(),
            m_headingHorizontal.getAsDouble(),
            m_headingVertical.getAsDouble());

    // Prevent Movement After Auto
    if (m_initRotation) {
      if (m_headingHorizontal.getAsDouble() == 0 && m_headingVertical.getAsDouble() == 0) {
        // Get the curretHeading
        Rotation2d firstLoopHeading = m_swerveSubsystem.getHeading();

        // Set the Current Heading to the desired Heading
        desiredSpeeds =
            m_swerveSubsystem.getTargetSpeeds(
                0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
      }
      // Dont Init Rotation Again
      m_initRotation = false;
    }

    // Limit velocity so we don't fall over
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation =
        SwerveMath.limitVelocity(
            translation,
            m_swerveSubsystem.getFieldVelocity(),
            m_swerveSubsystem.getPose(),
            Constants.SWERVE_LOOP_TIME,
            Constants.ROBOT_MASS,
            List.of(Constants.CHASSIS),
            m_swerveSubsystem.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    m_swerveSubsystem.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
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
