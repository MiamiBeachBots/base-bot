// Copyright (c) Max Aitel, Jack Nelson & Miami Beach Bots

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  // motors
  private final WPI_VictorSPX m_clawMotor;
  // limit switches
  private final DigitalInput m_clawLimitSwitchOpen;
  private final DigitalInput m_clawLimitSwitchClose;
  // constants
  private final double movementSpeed = 0.1;
  private final double closeSpeed = 0.01;

  public ClawSubsystem() {
    // claw go grabby grabby
    m_clawMotor = new WPI_VictorSPX(CANConstants.ARMCLAWMOTORID);
    m_clawMotor.setInverted(false); // invert if needed here.
    // limit switches
    m_clawLimitSwitchOpen = new DigitalInput(Constants.LSWITCHCLAWOPEN);
    m_clawLimitSwitchClose = new DigitalInput(Constants.LSWITCHCLAWCLOSE);
  }

  public boolean clawOpen() {
    if (!m_clawLimitSwitchOpen.get()) {
      m_clawMotor.set(-movementSpeed);
      return false;
    } else {
      m_clawMotor.set(0.0);
      return true;
    }
  }

  public void clawBasic(double speed) {
    m_clawMotor.set(speed);
  }

  public boolean clawClose() {
    if (!m_clawLimitSwitchClose.get()) {
      m_clawMotor.set(movementSpeed);
      return false;
    } else {
      m_clawMotor.set(closeSpeed);
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_clawLimitSwitchOpen.get()) {
      SmartDashboard.putBoolean("Claw Open", true);
    } else if (m_clawLimitSwitchClose.get()) {
      SmartDashboard.putBoolean("Claw Open", false);
    } else {
      System.out.println("Claw is moving between states");
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
