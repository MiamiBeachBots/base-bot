// Copyright (c) Max Aitel & Miami Beach Bots

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  // motors
  private final CANSparkMax m_extMotor;

  private final CANSparkMax m_elevatorMotor;
  // limit switches
  private final DigitalInput m_extLimitSwitchFront;
  private final DigitalInput m_extLimitSwitchBack;

  public ArmSubsystem() {
    // arm extention/x motor
    m_extMotor = new CANSparkMax(CANConstants.ARMEXTENSIONMOTORID, MotorType.kBrushless);
    // arm up/y motor
    m_elevatorMotor = new CANSparkMax(CANConstants.ARMELEVATORMOTORID, MotorType.kBrushless);
    // limit switches
    m_extLimitSwitchFront = new DigitalInput(Constants.LSWITCHEXTFRONT);
    m_extLimitSwitchBack = new DigitalInput(Constants.LSWITCHEXTBACK);
  }

  public void armExtend(double axisValue) {
    if (!m_extLimitSwitchFront.get() || !m_extLimitSwitchBack.get()) {
      // move with value and speed
    }
  }

  public void armElevate(boolean direction) {
    // arm up down
    System.out.println("Arm is Elevating");
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
