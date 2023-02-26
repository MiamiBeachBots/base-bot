// Copyright (c) Max Aitel, Jack Nelson & Miami Beach Bots

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

public class ExtensionSubsystem extends SubsystemBase {
  /** Creates a new ExtensionSubsystem. */
  // extension motor
  private final CANSparkMax m_extMotor;

  // limit switches
  private final DigitalInput m_extLimitSwitchFront;
  private final DigitalInput m_extLimitSwitchBack;

  public ExtensionSubsystem() {
    // arm extention/x motor
    m_extMotor = new CANSparkMax(CANConstants.ARMEXTENSIONMOTORID, MotorType.kBrushless);
    // limit switches
    m_extLimitSwitchFront = new DigitalInput(Constants.LSWITCHEXTFRONT);
    m_extLimitSwitchBack = new DigitalInput(Constants.LSWITCHEXTBACK);
    // configure pid & motor
  }

  public void armExtend(double axisValue) {
    if (!m_extLimitSwitchFront.get() || !m_extLimitSwitchBack.get()) {
      // move with value and speed
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
