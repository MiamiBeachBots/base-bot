package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */

public class ShooterState {

  public boolean isLoaded = false;
  public boolean isLowered = false;

  public enum ShooterMode {
    DEFAULT,
    SOURCE,
    AMP,
    SPEAKER,
    TRAP,
    STOP
  };

  public ShooterMode mode = ShooterMode.DEFAULT;
  public boolean axisEnabled = false;
  public boolean shooting = false;

  public ShooterState() {}

  public void setLoaded() {
    isLoaded = true;
  }

  public void setMode(ShooterMode newMode) {
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
    /*
    Example:
    switch (mode) {
      case SOURCE: // TODO
        return Constants.SHOOTERSOURCE;
      case AMP:
        return Constants.SHOOTERAMP;
      case SPEAKER:
        return Constants.SHOOTERSPEAKER;
      case TRAP:
        return Constants.SHOOTERTRAP;
      default:
        return Constants.SHOOTERDEFAULT;
    }
    */
    return 0;
  }

  /**
   * Updates the values on the SmartDashboard related to the shooter state. This method puts the
   * values of various shooter state variables onto the SmartDashboard. The variables include
   * whether the manual arm mode is enabled, the current arm mode, whether the shooter is loaded,
   * whether the arm is lowered, and whether the arm is shooting.
   */
  public void updateDash() {
    SmartDashboard.putBoolean("Manual Arm Mode Enabled", axisEnabled);
    SmartDashboard.putString("Arm Mode", mode.name());
    SmartDashboard.putBoolean("Loaded", isLoaded);
    SmartDashboard.putBoolean("Lowered", isLowered);
    SmartDashboard.putBoolean("Arm Shooting", shooting);
  }
}
