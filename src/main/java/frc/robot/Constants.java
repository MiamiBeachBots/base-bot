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
    public static final int MOTORFRONTRIGHTID = 14;
    public static final int MOTORBACKRIGHTID = 13;
    public static final int MOTORFRONTLEFTID = 12;
    public static final int MOTORBACKLEFTID = 11;
    /// Arm Motors
    public static final int MOTORARMMAINID = 21;
    /// Shooter Motors
    public static final int MOTORSHOOTERLEFTID = 31;
    public static final int MOTORSHOOTERRIGHTID = 32;
    /// Lifter Motors
    public static final int MOTORLIFTERLEFTID = 41;
    public static final int MOTORLIFTERRIGHTID = 42;
    /// Elevator Motors
    public static final int MOTORELEVATORID = 51;
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
  public static final double LIFTERSPEED = 0.5;

  // USB Devices
  public static final int CONTROLLERUSBINDEX = 0;
  public static final int FLIGHTSTICKUSBINDEX = 1;
  // On-Controller joystick deadzone
  public static final double CONTROLLERDEADZONE = 0.1;

  // Game Controller Buttons
  // Now In RobotContainer as Native Triggers.

  // Joystick buttons
  public static final int TRIGGER = 1;
  public static final int AIMBUTTON = 12;

  // Analog Ports
  /// Ultrasonic Sensors and ports.
  public static final int ULTRASONICSHOOTERPORT = 0;

  // Digital Ports
  public static final int EXAMPLELIMITSWITCH = 0;

  // Shooter Angles

  // Camera Constants
  public static final class PoseCamera1 {
    public static final String name = "pose1";
    // XYZ
    private static final double XLocation = Units.inchesToMeters(6);
    private static final double YLocation = Units.inchesToMeters(9.3);
    private static final double ZLocation = Units.inchesToMeters(10.5);
    // ROTATION
    public static final double roll = Units.degreesToRadians(90);
    public static final double pitch = Units.degreesToRadians(0.0);
    public static final double yaw = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(XLocation, YLocation, ZLocation), new Rotation3d(roll, pitch, yaw));
  }

  public static final class PoseCamera2 {
    public static final String name = "pose2";
    // XYZ
    private static final double XLocation = Units.inchesToMeters(6);
    private static final double YLocation = Units.inchesToMeters(9.3);
    private static final double ZLocation = Units.inchesToMeters(10.5);
    // ROTATION
    public static final double roll = Units.degreesToRadians(90);
    public static final double pitch = Units.degreesToRadians(0.0);
    public static final double yaw = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(XLocation, YLocation, ZLocation), new Rotation3d(roll, pitch, yaw));
  }

  public static final class TargetingCamera1 {
    public static final String name = "targeting1";
    // XYZ
    private static final double XLocation = Units.inchesToMeters(6);
    private static final double YLocation = Units.inchesToMeters(9.3);
    private static final double ZLocation = Units.inchesToMeters(10.5);
    // ROTATION
    public static final double roll = Units.degreesToRadians(90);
    public static final double pitch = Units.degreesToRadians(0.0);
    public static final double yaw = Units.degreesToRadians(0);

    public static final Transform3d location =
        new Transform3d(
            new Translation3d(XLocation, YLocation, ZLocation), new Rotation3d(roll, pitch, yaw));
  }
}
