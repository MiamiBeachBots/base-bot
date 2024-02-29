package frc.robot.utils;

public final class HelperFunctions {

  public static boolean inDeadzone(double value, double deadzone) {
    return Math.abs(value) < deadzone;
  }
}
