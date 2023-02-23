// Copyright (c) Max Aitel & Miami Beach Bots

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  // motors
  private final CANSparkMax m_extMotor;

  private final CANSparkMax m_elevatorMotor;
  private final WPI_VictorSPX m_clawMotor;
  // limit switches
  private final DigitalInput m_extLimitSwitchFront;
  private final DigitalInput m_extLimitSwitchBack;
  private final DigitalInput m_clawLimitSwitchOpen;
  private final DigitalInput m_clawLimitSwitchClose;

  public ArmSubsystem() {
    // arm extention/x motor
    m_extMotor = new CANSparkMax(CANConstants.ARMEXTENSIONMOTORID, MotorType.kBrushless);
    // arm up/y motor
    m_elevatorMotor = new CANSparkMax(CANConstants.ARMELEVATORMOTORID, MotorType.kBrushless);
    // claw go grabby grabby
    m_clawMotor = new WPI_VictorSPX(CANConstants.ARMCLAWMOTORID);
    // limit switches
    m_extLimitSwitchFront = new DigitalInput(Constants.LSWITCHEXTFRONT);
    m_extLimitSwitchBack = new DigitalInput(Constants.LSWITCHEXTBACK);
    m_clawLimitSwitchOpen = new DigitalInput(Constants.LSWITCHCLAWOPEN);
    m_clawLimitSwitchClose = new DigitalInput(Constants.LSWITCHCLAWCLOSE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armExtension(double value, double speed) {
    if (m_extLimitSwitchFront.get() != true || m_extLimitSwitchBack.get() != true) {
      // move with value and speed
    }
  }

  public void armElevator(double value, double speed) {
    // arm up down
  }

  public void clawOpen() {
    if (m_clawLimitSwitchOpen.get() != true) { // going to error until we get limit switch ids set
      m_clawMotor.set(0.1);
    }
    else {
      m_clawMotor.set(0.0);
    }
  }

  public void clawClose() {
    if (m_clawLimitSwitchClose.get() != true) { // going to error until we get limit switch ids set
      m_clawMotor.set(0.1);
    }
    else {
      m_clawMotor.set(0.01);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
