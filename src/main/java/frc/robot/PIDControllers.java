package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PIDControllers {

  // false when inactive, true when active / a target is set.
  public static boolean turnControllerEnabled = false;
  public static double turnRotateToAngleRate; // This value will be updated by the PID Controller
  // pid controller for "RotateToAngle"
  public static final ProfiledPIDController m_balanceController =
      new ProfiledPIDController(
          Constants.PIDConstants.BalancePIDConstants.balance_P,
          Constants.PIDConstants.BalancePIDConstants.balance_I,
          Constants.PIDConstants.BalancePIDConstants.balance_D,
          new TrapezoidProfile.Constraints(
              Constants.PIDConstants.BalancePIDConstants.MaxBalanceRateDegPerS,
              Constants.PIDConstants.BalancePIDConstants.MaxBalanceAccelerationDegPerSSquared));

  // false when inactive, true when active / a target is set.
  public static boolean distanceControllerEnabled = false;
  public static double distanceThrottleRate; // This value will be updated by the PID Controller
  // pid controller for "MoveDistance"
  public static final ProfiledPIDController m_distanceController =
      new ProfiledPIDController(
          Constants.PIDConstants.DistancePIDConstants.distance_P,
          Constants.PIDConstants.DistancePIDConstants.distance_I,
          Constants.PIDConstants.DistancePIDConstants.distance_D,
          new TrapezoidProfile.Constraints(
              Constants.PIDConstants.DistancePIDConstants.distanceMaxSpeed,
              Constants.PIDConstants.DistancePIDConstants.distanceMaxAcceleration));

  // false when inactive, true when active / a target is set.
  public static boolean balanceControllerEnabled = false;
  public static double balanceThrottleRate; // This value will be updated by the PID Controller
  // pid controller for balanceCorrection

  public static final ProfiledPIDController m_turnController =
      new ProfiledPIDController(
          Constants.PIDConstants.TurnPIDConstants.turn_P,
          Constants.PIDConstants.TurnPIDConstants.turn_I,
          Constants.PIDConstants.TurnPIDConstants.turn_D,
          new TrapezoidProfile.Constraints(
              Constants.PIDConstants.TurnPIDConstants.MaxTurnRateDegPerS,
              Constants.PIDConstants.TurnPIDConstants.MaxTurnAccelerationDegPerSSquared));
}
