package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The DriveConstants class has all of the constants needed for autonomous / gyro assisted movement
 * of the robot. This class is kept seperate as its values change quite a lot. Odemetry Is used to
 * track the position and state of the robot. Kinemtatics is used to calculate how much power to
 * apply to each motor.
 */
public final class DriveConstants {
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // meters
  public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2;
  public static final double kTrackwidthMeters = 0.527;
  // this is not used and is handled by the rev encoder.
  public static final double PULSES_PER_REV = 1;
  public static final double GEAR_RATIO = 8.46; // 8.46:1
  // basically converted from rotations to to radians to then meters using the wheel diameter.
  // the diameter is already *2 so we don't need to multiply by 2 again.
  public static final double POSITION_CONVERSION_RATIO =
      (Math.PI * WHEEL_DIAMETER) / PULSES_PER_REV / GEAR_RATIO;
  public static final double VELOCITY_CONVERSION_RATIO = POSITION_CONVERSION_RATIO / 60;
  // Kinematic constants

  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  // Feed Forward Constants
  public static final double ksDriveVolts = 0.015014;
  public static final double kvDriveVoltSecondsPerMeter = 2.4799;
  public static final double kaDriveVoltSecondsSquaredPerMeter = 0.48513;
  // For SIM
  public static final double kvDriveVoltSecondsPerMeterAngular = 1.2;
  public static final double kaDriveVoltSecondsSquaredPerMeterAngular = 0.2;
  // Max speed Constants
  public static final double kMaxOutputDrive = 0.8;
  public static final double kMinOutputDrive = -0.8;
  // Feed Back / PID Constants
  public static final double kPDriveVel = 0.0025097;
  public static final double kIDriveVel = 0.0;
  public static final double kDDriveVel = 0.0;
  public static final double kIzDriveVel = 0.0; // error before integral takes effect

  public static final double kPDrivePos = 3.2973;
  public static final double kIDrivePos = 0.0;
  public static final double kDDrivePos = 0.40434;
  public static final double kIzDrivePos = 0.0; // error before integral takes effect
  // Helper class that converts a chassis velocity (dx and dtheta components) to left and right
  // wheel velocities for a differential drive.
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);

  // Default path config from path planning app
  public static RobotConfig autoConfig;

  static {
    try {
      autoConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  // Motor Controller PID Slots
  public static final ClosedLoopSlot kDrivetrainVelocityPIDSlot = ClosedLoopSlot.kSlot0;
  public static final ClosedLoopSlot kDrivetrainPositionPIDSlot = ClosedLoopSlot.kSlot1;

  public final class OnTheFly {
    // On the fly path planning constants
    public static final double kMaxVelocity = 2; // m/s
    public static final double kMaxAcceleration = 1.5; // m/s^2
    public static final double kMaxAngularVelocity = 360; // deg/s
    public static final double kMaxAngularAcceleration = 300; // deg/s^2
    public static final double kNominalVoltage = 12; // V
    public static final PathConstraints kPathConstraints =
        new PathConstraints(
            kMaxVelocity,
            kMaxAcceleration,
            kMaxAngularVelocity,
            kMaxAngularAcceleration,
            kNominalVoltage);
  }

  public final class OnTheFlyReduced {
    // On the fly path planning constants
    public static final double kMaxVelocity = 1; // m/s
    public static final double kMaxAcceleration = 0.5; // m/s^2
    public static final double kMaxAngularVelocity = 360; // deg/s
    public static final double kMaxAngularAcceleration = 300; // deg/s^2
    public static final double kNominalVoltage = 12; // V
    public static final PathConstraints kPathConstraints =
        new PathConstraints(
            kMaxVelocity,
            kMaxAcceleration,
            kMaxAngularVelocity,
            kMaxAngularAcceleration,
            kNominalVoltage);
  }
}
