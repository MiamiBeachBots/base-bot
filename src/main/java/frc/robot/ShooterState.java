package frc.robot;

/*
 * ShooterState.java
 * Tracks the status of a ring loaded in the shooter
 */
public class ShooterState {
  public boolean isLoaded = false;
  public boolean isLowered = false;

  public ShooterState() {}

  public void setLoaded() {
    isLoaded = true;
  }

  public void setFired() {
    isLoaded = false;
    isLowered = false;
  }

  public void setLowered() {
    isLowered = true;
  }
}
