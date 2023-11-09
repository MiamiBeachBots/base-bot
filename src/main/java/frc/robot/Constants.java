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
  }

  public final class PIDConstants {
    public final class TurnPIDConstants {
      // Angle PID / RotateToAngle
      public static final double turn_P = 0.1;
      public static final double turn_I = 0.00;
      public static final double turn_D = 0.00;
      public static final double MaxTurnRateDegPerS = 100;
      public static final double MaxTurnAccelerationDegPerSSquared = 300;
      public static final double TurnToleranceDeg = 3; // max diff in degrees
      public static final double TurnRateToleranceDegPerS = 10; // degrees per second
    }

    public final class DistancePIDConstants {
      // Distance PID / MoveDistance
      public static final double distance_P = 0.1;
      public static final double distance_I = 0.00;
      public static final double distance_D = 0.00;
      public static final double distanceMaxSpeed = 1; // m/s
      public static final double distanceMaxAcceleration = 2; // m/s^2
      public static final double DistanceTolerance = 0.01; // max diff in meters
      public static final double DistanceSpeedTolerance = 0.1; // ignore if velocity is below. (m)
    }

    public final class BalancePIDConstants {
      // Balance PID / AutoBalance
      public static final double balance_P = 0.0625; // 1/16
      public static final double balance_I = 0.00;
      public static final double balance_D = 0.00;
      public static final double MaxBalanceRateDegPerS = 10;
      public static final double MaxBalanceAccelerationDegPerSSquared = 20;
      public static final double BalanceToleranceDeg = 2; // max diff in degrees
    }
  }

  public static final double MAX_SPEED = 0.8;

  // USB Devices
  public static final int CONTROLLERUSBINDEX = 0;
  public static final int FLIGHTSTICKUSBINDEX = 1;

  // Game Controller Buttons
  // Now In RobotContainer as Native Triggers.

  // Joystick buttons
  public static final int AIMBUTTON = 12;

  // Analog Ports
  /// Ultrasonic Sensors and ports.
  public static final int ULTRASONIC1PORT = 0;

  // Digital Ports
  /// Encoder Ports
  public static final int DRIVEENCODERLEFTA = 0;
  public static final int DRIVEENCODERLEFTB = 1;
  public static final int DRIVEENCODERRIGHTA = 2;
  public static final int DRIVEENCODERRIGHTB = 3;
}
