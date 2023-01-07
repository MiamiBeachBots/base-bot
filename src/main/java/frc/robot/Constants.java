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
  /// PWM ports until we switch to ring.
  // Drive Train Motors
  public static final int MOTORFRONTRIGHTCHANNEL = 4;
  public static final int MOTORFRONTLEFTCHANNEL = 1;
  public static final int MOTORBACKRIGHTCHANNEL = 3;
  public static final int MOTORBACKLEFTCHANNEL = 2;
  // Elevator Motors
  public static final int ELEVATORMOTORCHANNEL = 0;
  // Intake Motors
  public static final int INTAKEMOTORFRONTCHANNEL = 5;
  public static final int INTAKEMOTORBACKCHANNEL = 8;
  // Shooter Motors and Servos
  public static final int SHOOTERMOTOR1CHANNEL = 7;
  public static final int SHOOTERMOTOR2CHANNEL = 6;
  public static final int SHOOTERSERVOCHANNEL = 9;
  // Ultrasonic Sensors and ports.
  public static final int ULTRASONIC1PORT = 0;
}
