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
  private final DigitalInput m_clawLimitSwitch;

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
    m_clawLimitSwitch = new DigitalInput(Constants.LSWITCHCLAW);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armExtension(double value, double speed) {
    // arm extend crab
  }

  public void armElevator(double value, double speed) {
    // arm go up and down like a funi dance
  }

  public void clawOpen() {
    // arm go droppy droppy
  }

  public void clawClose() {
    // arm go grabby grabby
  }

  public void clawHold() {
    // arm go holdy holdy
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
