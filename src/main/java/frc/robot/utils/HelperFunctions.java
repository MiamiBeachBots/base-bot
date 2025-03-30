package frc.robot.utils;

public final class HelperFunctions {

  public static boolean inDeadzone(double value, double deadzone) {
    return Math.abs(value) < deadzone;
  }

  public static boolean inRange(double target, double measurment, double deadzone) {
    return Math.abs((target - measurment)) < deadzone;
  }
}
