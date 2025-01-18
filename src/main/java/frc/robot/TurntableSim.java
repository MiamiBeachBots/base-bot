// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated single jointed turntable mechanism. */
public class TurntableSim extends LinearSystemSim<N2, N1, N2> {
  // The gearbox for the turntable.
  private final DCMotor m_gearbox;

  // The gearing between the motors and the output.
  private final double m_gearing;

  /**
   * Creates a simulated turntable mechanism.
   *
   * @param plant The linear system that represents the turntable. This system can be created with
   *     {@link
   *     edu.wpi.first.math.system.plant.LinearSystemId#createSingleJointedturntableSystem(DCMotor,
   *     double, double)}.
   * @param gearbox The type of and number of motors in the turntable gearbox.
   * @param gearing The gearing of the turntable (numbers greater than 1 represent reductions).
   * @param simulateGravity Whether gravity should be simulated or not.
   * @param startingAngleRads The initial position of the turntable simulation in radians.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  @SuppressWarnings("this-escape")
  public TurntableSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      double startingAngleRads,
      double... measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
    setState(startingAngleRads, 0.0);
  }

  /**
   * Creates a simulated mechanism.
   *
   * @param gearbox The type of and number of motors in the turntable gearbox.
   * @param gearing The gearing of the turntable (numbers greater than 1 represent reductions).
   * @param kv kv values for the turntable.
   * @param ka ka values for the turntable.
   * @param startingAngleRads The initial position of the turntable simulation in radians.
   * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
   *     noise is desired. If present must have 1 element for position.
   */
  public TurntableSim(
      DCMotor gearbox,
      double gearing,
      double kv,
      double ka,
      double startingAngleRads,
      double... measurementStdDevs) {
    this(
        LinearSystemId.identifyPositionSystem(kv, ka),
        gearbox,
        gearing,
        startingAngleRads,
        measurementStdDevs);
  }

  /**
   * Sets the turntable's state. The new angle will be limited between the minimum and maximum
   * allowed limits.
   *
   * @param angleRadians The new angle in radians.
   * @param velocityRadPerSec The new angular velocity in radians per second.
   */
  public final void setState(double angleRadians, double velocityRadPerSec) {
    setState(VecBuilder.fill(angleRadians, velocityRadPerSec));
  }

  /**
   * Returns the current turntable angle.
   *
   * @return The current turntable angle.
   */
  public double getAngleRads() {
    return getOutput(0);
  }

  /**
   * Returns the current turntable velocity.
   *
   * @return The current turntable velocity.
   */
  public double getVelocityRadPerSec() {
    return getOutput(1);
  }

  /**
   * Returns the turntable current draw.
   *
   * @return The turntable current draw.
   */
  public double getCurrentDrawAmps() {
    // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
    // spinning 10x faster than the output
    var motorVelocity = m_x.get(1, 0) * m_gearing;
    return m_gearbox.getCurrent(motorVelocity, m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the turntable.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
    clampInput(RobotController.getBatteryVoltage());
  }
}
