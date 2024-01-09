package frc.robot;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */
public class ShooterState {
  public boolean isLoaded = false;
  public boolean isLowered = false;

  public ShooterState() {}

  public void loaded() {
    isLoaded = true;
  }

  public void fired() {
    isLoaded = false;
    isLowered = false;
  }

  public void lowered() {
    isLowered = true;
  }
}
