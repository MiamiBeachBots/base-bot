package frc.robot;

public final class PIDConstants {
  public static final class balancePID {
    // Balance PID / AutoBalance
    public static final double balance_P = 0.0625; // 1/16
    public static final double balance_I = 0.00;
    public static final double balance_D = 0.00;
    public static final double MaxBalanceRateDegPerS = 10;
    public static final double MaxBalanceAccelerationDegPerSSquared = 20;
    public static final double BalanceToleranceDeg = 2; // max diff in degrees
  }

  public static final class distancePID {
    // Distance PID / MoveDistance
    public static final double distance_P = 0.1;
    public static final double distance_I = 0.00;
    public static final double distance_D = 0.00;
    public static final double distanceMaxSpeed = 1; // m/s
    public static final double distanceMaxAcceleration = 2; // m/s^2
    public static final double DistanceTolerance = 0.01; // max diff in meters
    public static final double DistanceSpeedTolerance = 0.1; // ignore if velocity is below. (m)
  }

  public static final class turnPID {
    // Angle PID / RotateToAngle
    public static final double turn_P = 0.1;
    public static final double turn_I = 0.00;
    public static final double turn_D = 0.00;
    public static final double MaxTurnRateDegPerS = 100;
    public static final double MaxTurnAccelerationDegPerSSquared = 50;
    public static final double TurnToleranceDeg = 3; // max diff in degrees
    public static final double TurnRateToleranceDegPerS = 10; // degrees per second
    // false when inactive, true when active / a target is set.
  }
}
