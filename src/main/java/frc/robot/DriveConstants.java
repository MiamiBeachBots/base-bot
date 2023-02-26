package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The DriveConstants class has all of the constants needed for autonomous / gyro assisted movement
 * of the robot. This class is kept seperate as its values change quite a lot.
 */
public final class DriveConstants {
  // general drive constants
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // meters
  public static final double PULSES_PER_REV = 2048; // resolution of encoder
  public static final double GEAR_RATIO = 1;
  public static final double DISTANCE_PER_PULSE =
      (Math.PI * WHEEL_DIAMETER) / PULSES_PER_REV / GEAR_RATIO;
  // Kinematic constants

  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final double Ks = 0.74056; // volts
  public static final double Kv = 2.4474; // VoltSecondsPerMeter
  public static final double Ka = 0.45481; // VoltSecondsSquaredPerMeter
  public static final double kPDriveVel = 3.5479;
  // differental drive constants
  public static final double kTrackwidthMeters = Units.inchesToMeters(21.0);
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);
  // MAX Acceleration & Velocity
  public static final double kMaxSpeed = 3; // MetersPerSecond
  public static final double kMaxAcceleration = 1; // MetersPerSecondSquared
  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  // Final Kinematic Configs
  public static final SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(Ks, Kv, Ka);
  // Create a voltage constraint to ensure we don't accelerate too fast
  public static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(FeedForward, kDriveKinematics, 10);

  // Create config for trajectory
  public static final TrajectoryConfig driveTrajectoryConfig =
      new TrajectoryConfig(kMaxSpeed, kMaxAcceleration)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);
}
