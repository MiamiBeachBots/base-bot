// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** This class is for any devices that connect to the CAN system. */
  public static final class CANConstants {
    // CAN Bus Devices
    /// Drive Train Motors
    public static final int MOTOR_FRONT_RIGHT_ID = 14;
    public static final int MOTOR_BACK_RIGHT_ID = 13;
    public static final int MOTOR_FRONT_LEFT_ID = 12;
    public static final int MOTOR_BACK_LEFT_ID = 11;
    /// Arm Motors
    public static final int MOTOR_ARM_MAIN_ID = 21;
    /// Shooter Motors
    public static final int MOTOR_SHOOTER_LEFT_ID = 31;
    public static final int MOTOR_SHOOTER_RIGHT_ID = 32;
    /// Lifter Motors
    public static final int MOTOR_LIFTER_LEFT_ID = 41;
    public static final int MOTOR_LIFTER_RIGHT_ID = 42;
    /// Elevator Motors
    public static final int MOTOR_ELEVATOR_ID = 51;
    /// Turntable Motor
    public static final int MOTOR_TURNTABLE_ID = 61;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double MAX_SPEED = 0.8;
  public static final double LIFTER_SPEED = 0.5;

  // USB Devices
  public static final int CONTROLLER_USB_INDEX = 0;
  public static final int FLIGHTSTICK_USB_INDEX = 1;
  // On-Controller joystick deadzone
  public static final double CONTROLLER_DEAD_ZONE = 0.1;

  // Game Controller Buttons
  // Now In RobotContainer as Native Triggers.

  // Joystick buttons
  public static final int TRIGGER = 1;
  public static final int AIM_BUTTON = 12;

  // Analog Ports
  /// Ultrasonic Sensors and ports.
  public static final int ULTRASONIC_SHOOTER_PORT = 0;

  // Digital Ports
  public static final int EXAMPLE_LIMIT_SWITCH = 0;

  // Shooter Angles

  // Camera Constants
  public static final class PoseCamera1 {
    public static final String name = "pose1";
    // XYZ
    private static final double xLocation = Units.inchesToMeters(6);
    private static final double yLocation = Units.inchesToMeters(9.3);
    private static final double zLocation = Units.inchesToMeters(10.5);
    // ROTATION
    public static final double roll = Units.degreesToRadians(90);
    public static final double pitch = Units.degreesToRadians(0.0);
    public static final double yaw = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(xLocation, yLocation, zLocation), new Rotation3d(roll, pitch, yaw));
  }

  public static final class PoseCamera2 {
    public static final String name = "pose2";
    // XYZ
    private static final double xLocation = Units.inchesToMeters(6);
    private static final double yLocation = Units.inchesToMeters(9.3);
    private static final double zLocation = Units.inchesToMeters(10.5);
    // ROTATION
    public static final double roll = Units.degreesToRadians(90);
    public static final double pitch = Units.degreesToRadians(0.0);
    public static final double yaw = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(xLocation, yLocation, zLocation), new Rotation3d(roll, pitch, yaw));
  }

  public static final class TargetingCamera1 {
    public static final String name = "targeting1";
    // XYZ
    private static final double xLocation = Units.inchesToMeters(6);
    private static final double yLocation = Units.inchesToMeters(9.3);
    private static final double zLocation = Units.inchesToMeters(10.5);
    // ROTATION
    public static final double roll = Units.degreesToRadians(90);
    public static final double pitch = Units.degreesToRadians(0.0);
    public static final double yaw = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(xLocation, yLocation, zLocation), new Rotation3d(roll, pitch, yaw));
  }
}
