package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */

public class ShooterState {
  public final class ShooterMode {
    public static final int DEFAULT = 0;
    public static final int SOURCE = 1;
    public static final int AMP = 2;
    public static final int SPEAKER = 3;
  }

  public boolean isLoaded = false;
  public boolean isLowered = false;
  public int mode = ShooterMode.SOURCE;
  public boolean axisEnabled = false;
  public boolean shooting = false;

  public ShooterState() {}

  public void setLoaded() {
    isLoaded = true;
  }

  public void setMode(int newMode) {
    mode = newMode;
  }

  public void toggleAxis() {
    axisEnabled = !axisEnabled;
  }

  public void setFired() {
    isLoaded = false;
    isLowered = false;
    mode = ShooterMode.SOURCE;
  }

  public void setLowered() {
    isLowered = true;
    mode = ShooterMode.DEFAULT;
  }

  public double getShooterSpeed() {
    switch (mode) {
      case ShooterMode.SOURCE: // TODO
        return Constants.SHOOTERSOURCE;
      case ShooterMode.AMP:
        return Constants.SHOOTERAMP;
      case ShooterMode.SPEAKER:
        return Constants.SHOOTERSPEAKER;
      default:
        return Constants.SHOOTERDEFAULT;
    }
  }

  public void updateDash() {
    SmartDashboard.putBoolean("Manual Arm Mode Enabled", axisEnabled);
    SmartDashboard.putNumber("Arm Mode", mode);
    SmartDashboard.putBoolean("Loaded", isLoaded);
    SmartDashboard.putBoolean("Lowered", isLowered);
    SmartDashboard.putBoolean("Arm Shooting", shooting);
  }
}
