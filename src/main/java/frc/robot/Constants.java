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

    /// Elevator Motors
    public static final int MOTOR_ELEVATOR_LEFT_ID = 21;
    public static final int MOTOR_ELEVATOR_RIGHT_ID = 22;

    /// Arm Motors
    public static final int MOTOR_ARM_MAIN_ID = 31;

    /// Shooter Motors
    public static final int MOTOR_SHOOTER_LEFT_ID = 41;
    public static final int MOTOR_SHOOTER_RIGHT_ID = 42;

    /// Lifter Motors
    public static final int MOTOR_LIFTER_LEFT_ID = -1;
    public static final int MOTOR_LIFTER_RIGHT_ID = -1;
  }

  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Max speeds
  public static final double MAX_SPEED = 0.8;
  public static final double MAX_EXTENDED_SPEED = 0.3;
  public static final double MAX_SHOOTER_SPEED = 0.9;
  public static final double LIFTER_SPEED = 0.5;

  public static final double ARM_ANGLE_OFFSET = Units.degreesToRadians(180); // Also max angle
  public static final double ELEVATOR_STARTING_HEIGHT_INCHES = 13;
  public static final double ELEVATOR_STARTING_HEIGHT =
      Units.inchesToMeters(ELEVATOR_STARTING_HEIGHT_INCHES);
  public static final double ELEVATOR_MAX_HEIGHT = 1.88; // METERS
  // Radians (based on sensor) (starting angle straight down)
  public static final double ARM_START_OFFSET = 0.2799;
  public static final double ARM_ZERO_ENCODER_OFFSET = 0.5872441; // Touching frame

  // USB Devices
  public static final int CONTROLLER_USB_INDEX = 0;
  public static final int FLIGHTSTICK_USB_INDEX = 1;
  // On-Controller joystick deadzone
  public static final double CONTROLLER_DEAD_ZONE = 0.1;

  // Game Controller Buttons
  // Now In RobotContainer as Native Triggers.

  // Joystick buttons
  public static final int TRIGGER = 1;
  public static final int DEFAULT_BUTTON = 2;
  public static final int TROUGH_BUTTON = 7;
  public static final int REEFT2_BUTTON = 8;
  public static final int REEFT3_BUTTON = 9;
  public static final int REEFT4_BUTTON = 10;
  public static final int BARGE_BUTTON = 12;
  public static final int INTAKE_BUTTON = 11;
  public static final int AIM_BUTTON = 3;

  // Analog Ports
  /// Ultrasonic Sensors and ports.
  public static final int ULTRASONIC_SHOOTER_PORT = 0;

  // Digital Ports
  public static final int EXAMPLE_LIMIT_SWITCH = 0;

  // Shooter Angles
  // Camera Constants
  public static final class PoseCamera1 {
    public static final String NAME = "Pose1";
    // XYZ
    private static final double X_LOCATION = Units.inchesToMeters(2);
    private static final double Y_LOCATION = Units.inchesToMeters(-10.25);
    private static final double Z_LOCATION = Units.inchesToMeters(33.875);
    // ROTATION
    private static final double ROLL = Units.degreesToRadians(90);
    private static final double PITCH = Units.degreesToRadians(0.0);
    private static final double YAW = Units.degreesToRadians(17.5);

    public static final Transform3d LOCATION =
        new Transform3d(
            new Translation3d(X_LOCATION, Y_LOCATION, Z_LOCATION),
            new Rotation3d(ROLL, PITCH, YAW));
  }

  public static final class PoseCamera2 {
    public static final String NAME = "Pose2";
    // XYZ
    private static final double X_Location = Units.inchesToMeters(4.75);
    private static final double Y_Location = Units.inchesToMeters(9);
    private static final double Z_Location = Units.inchesToMeters(33.25);
    // ROTATION
    private static final double ROLL = Units.degreesToRadians(90);
    private static final double PITCH = Units.degreesToRadians(0.0);
    private static final double YAW = Units.degreesToRadians(12.5);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(X_Location, Y_Location, Z_Location),
            new Rotation3d(ROLL, PITCH, YAW));
  }

  public static final class TargetingCamera1 {
    public static final String NAME = "Targeting1";
    // XYZ
    private static final double X_LOCATION = Units.inchesToMeters(13.5);
    private static final double Y_LOCATION = Units.inchesToMeters(1);
    private static final double Z_LOCATION = Units.inchesToMeters(12.5);
    // ROTATION
    private static final double ROLL = Units.degreesToRadians(0);
    private static final double PITCH = Units.degreesToRadians(0.0);
    private static final double YAW = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(X_LOCATION, Y_LOCATION, Z_LOCATION),
            new Rotation3d(ROLL, PITCH, YAW));
  }

  public static final class AlgaeCamOffset {
    // XYZ, should be measured from camera
    private static final double X_Location = Units.inchesToMeters(12);
    private static final double Y_Location = Units.inchesToMeters(0);
    private static final double Z_Location = Units.inchesToMeters(0);
    // ROTATION
    private static final double ROLL = Units.degreesToRadians(0);
    private static final double PITCH = Units.degreesToRadians(0.0);
    private static final double YAW = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(X_Location, Y_Location, Z_Location),
            new Rotation3d(ROLL, PITCH, YAW));
  }
}
