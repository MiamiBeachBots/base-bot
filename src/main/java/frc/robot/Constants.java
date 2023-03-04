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
    public static final int MOTORFRONTRIGHTID = 11;
    public static final int MOTORBACKRIGHTID = 12;
    public static final int MOTORFRONTLEFTID = 13;
    public static final int MOTORBACKLEFTID = 14;
    /// Arm movement motors
    public static final int ARMEXTENSIONMOTORID = 21;
    public static final int ARMELEVATORMOTORID = 22;
    public static final int ARMCLAWMOTORID = 23;
  }

  public static final double MAX_SPEED = 0.8;

  // USB Devices
  public static final int CONTROLLERUSBINDEX = 0;
  public static final int FLIGHTSTICKUSBINDEX = 1;

  // Joystick buttons
  public static final int AIMBUTTON = 12;
  public static final int CLAWBUTTON = 1;
  public static final int ARMDOWNBUTTON = 3;
  public static final int ARMUPBUTTON = 4;
  // Analog Ports
  /// Ultrasonic Sensors and ports.
  public static final int ULTRASONIC1PORT = 0;

  // Digital Ports
  /// Encoder Ports
  public static final int DRIVEENCODERLEFTA = 0;
  public static final int DRIVEENCODERLEFTB = 1;
  public static final int DRIVEENCODERRIGHTA = 2;
  public static final int DRIVEENCODERRIGHTB = 3;
  /// Limit Switches
  public static final int LSWITCHCLAWOPEN = 4;
  public static final int LSWITCHCLAWCLOSE = 5;
  public static final int LSWITCHEXTFRONT = 6;
  public static final int LSWITCHEXTBACK = 7;
}
