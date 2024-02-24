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
    public static final int MOTORSHOOTERLEFTID = 31;
    public static final int MOTORSHOOTERRIGHTID = 32;
    /// Lifter Motors
    public static final int MOTORLIFTERLEFTID = 41;
    public static final int MOTORLIFTERRIGHTID = 42;
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
  public static final int ARMAMPBUTTON = 3;
  public static final int ARMSPEAKERBUTTON = 4;
  public static final int ARMDEFAULTBUTTON = 2;
  public static final int ARMLOADBUTTON = 6;
  public static final int ENABLEAXISBUTTON = 10;
  public static final int AIMBUTTON = 12;

  // Analog Ports
  /// Ultrasonic Sensors and ports.
  public static final int ULTRASONICSHOOTERPORT = 0;

  // Digital Ports

  // Shooter Angles
  public static final double ARMSTARTINGANGLE = 22.5; // WHY MaTH HURT
  public static final double ARMLOADANGLE = 45;
  public static final double ARMSPEAKERANGLE =
      70; // our start is like 40 degrees or something lol - ma
  public static final double ARMAMPANGLE = 50;
  // Shooter Speeds (M/s)
  public static final double SHOOTERSOURCE = -8.0;
  public static final double SHOOTERAMP = 3.0;
  public static final double SHOOTERSPEAKER = 8.0;
  public static final double SHOOTERDEFAULT = 8.0; // somewhere around 8 is the cap - ma
}
