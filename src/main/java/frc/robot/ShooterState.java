package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */

public class ShooterState {
  /** Class for presets */
  public static class ShooterMode {
    public final String name;
    public final double speed;
    public final double height;
    public final double angle;

    /**
     * @param Name Which preset is it
     * @param Speed m/s
     * @param Height inches
     * @param Angle degrees
     */
    public ShooterMode(String Name, double Speed, double Height, double Angle) {
      name = Name;
      speed = Speed;
      if (Height == 0) {
        height = 0;
      } else {
        height = Units.inchesToMeters(Height) - Constants.ELEVATOR_OFFSET;
      }

      if (Angle == 0) {
        angle = 0;
      } else {
        angle = Units.degreesToRadians(Angle) - Constants.ARM_ANGLE_OFFSET;
      }
    }
  }

  // TODO: Numbers
  public static class ShooterModes {
    public static final ShooterMode DEFAULT =
        new ShooterMode("Default", Constants.MAX_SHOOTER_SPEED, 0, 105);
    public static final ShooterMode INTAKE =
        new ShooterMode("Intake", -Constants.MAX_SHOOTER_SPEED, 0, -30);
    public static final ShooterMode PROCESSOR =
        new ShooterMode("Processor", Constants.MAX_SHOOTER_SPEED * 0.25, 0, -15);
    public static final ShooterMode TROUGH =
        new ShooterMode("Trough", Constants.MAX_SHOOTER_SPEED, 19, -15);
    public static final ShooterMode REEFT2 =
        new ShooterMode("ReefT2", Constants.MAX_SHOOTER_SPEED, 32, -30);
    public static final ShooterMode REEFT3 =
        new ShooterMode("ReefT3", Constants.MAX_SHOOTER_SPEED, 46.28, -30);
    public static final ShooterMode REEFT4 =
        new ShooterMode("ReefT4", Constants.MAX_SHOOTER_SPEED, 71.87, -60);
    public static final ShooterMode BARGE =
        new ShooterMode("Barge", Constants.MAX_SHOOTER_SPEED * 0.25, 78, 0);
  }
  ;

  public boolean isLoaded = true;
  public boolean isElevatorLowered = true;
  public boolean isArmResting = true; // Starting position
  public boolean isShooting = false;
  public boolean axisEnabled = false;
  private ShooterMode currentMode = ShooterModes.DEFAULT;
  public ShooterMode queuedMode = ShooterModes.DEFAULT;

  public ShooterState() {}

  public void setLoaded() {
    isLoaded = true;
  }

  public void setUnloaded() {
    isLoaded = false;
  }

  public ShooterMode getCurrentMode() {
    return currentMode;
  }

  private void setCurrentMode(ShooterMode newMode) {
    currentMode = newMode;
  }

  public void setQueuedMode(ShooterMode newMode) {
    queuedMode = newMode;
  }

  public void setArmResting(boolean isResting) {
    this.isArmResting = isResting;
  }

  public void startShooting() {
    isShooting = true;
  }

  public void stopShooting() {
    isShooting = false;
    // If intaking, and shooter is loaded, go to default
    if (getCurrentMode() == ShooterModes.INTAKE && isLoaded) {
      setCurrentMode(ShooterModes.DEFAULT);
      // After we finish shooting, go to default
    } else if (getCurrentMode() != ShooterModes.INTAKE && !isLoaded) {
      setCurrentMode(ShooterModes.DEFAULT);
    }
  }

  public void toggleAxis() {
    axisEnabled = !axisEnabled;
  }

  public void setElevatorLowered(boolean isElevatorLowered) {
    this.isElevatorLowered = isElevatorLowered;
  }

  public double getShooterSpeed() {
    return currentMode.speed;
  }

  /**
   * Updates the values on the SmartDashboard related to the shooter state. This method puts the
   * values of various shooter state variables onto the SmartDashboard. The variables include
   * whether the manual arm mode is enabled, the current arm mode, whether the shooter is loaded,
   * whether the arm is lowered, and whether the arm is shooting. It also adds things to the logs
   */
  public void StatePeriodic() {
    // Update SmartDashboard
    SmartDashboard.putBoolean("Manual Arm Mode Enabled", axisEnabled);
    SmartDashboard.putString("Current Mode", currentMode.name);
    SmartDashboard.putString("Queued Mode", queuedMode.name);
    SmartDashboard.putBoolean("Loaded", isLoaded);
    SmartDashboard.putBoolean("Elevator Lowered", isElevatorLowered);
    SmartDashboard.putBoolean("Resting", isArmResting);
    SmartDashboard.putBoolean("Arm Shooting", isShooting);
    // Add to log
    Logger.recordOutput("ArmStateManual", axisEnabled);
    Logger.recordOutput("ArmStateCurrentMode", currentMode.name);
    Logger.recordOutput("ArmStateQueuedMode", queuedMode.name);
    Logger.recordOutput("ArmStateLoaded", isLoaded);
    Logger.recordOutput("ArmStateResting", isArmResting);
    Logger.recordOutput("ArmStateShooting", isShooting);
    Logger.recordOutput("ElevatorStateIsLowered", isElevatorLowered);
  }
}
