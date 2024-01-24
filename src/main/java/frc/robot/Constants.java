// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot;

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
    public static final int MOTORSHOOTERID = 31;
  }

  public static final double MAX_SPEED = 0.8;

  // USB Devices
  public static final int CONTROLLERUSBINDEX = 0;
  public static final int FLIGHTSTICKUSBINDEX = 1;
  // On-Controller joystick deadzone
  public static final double CONTROLLERDEADZONE = 0.1;

  // Game Controller Buttons
  // Now In RobotContainer as Native Triggers.

  // Joystick buttons
  public static final int AIMBUTTON = 12;
  public static final int FIREBUTTON = 0;

  // Analog Ports
  /// Ultrasonic Sensors and ports.
  public static final int ULTRASONICSHOOTERPORT = 0;

  // Digital Ports
}
