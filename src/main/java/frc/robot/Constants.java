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
  public static final double MAX_SPEED = 0.8;

  // USB Devices
  public static final int CONTROLLERUSBINDEX = 0;
  public static final int FLIGHTSTICKUSBINDEX = 1;
  /// CAN Bus Devices
  // Drive Train Motors
  public static final int MOTORFRONTRIGHTID = 1;
  public static final int MOTORBACKRIGHTID = 2;
  public static final int MOTORFRONTLEFTID = 3;
  public static final int MOTORBACKLEFTID = 4;
  // Ultrasonic Sensors and ports.
  public static final int ULTRASONIC1PORT = 0;
  // Buttons
  public static final int SWAPCAMBUTTON = 1;
  public static final int AIMBUTTON = 12;
}
