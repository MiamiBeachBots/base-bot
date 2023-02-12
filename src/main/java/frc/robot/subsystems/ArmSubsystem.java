// Copyright (c) Max Aitel & Miami Beach Bots

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax m_extensionMotor;

  private final CANSparkMax m_elevatorMotor;

  private final CANSparkMax m_clawMotor;

  public ArmSubsystem() {
    // arm extention/x motor
    m_extensionMotor = new CANSparkMax(Constants.ARMEXTENSIONMOTORID, MotorType.kBrushless);
    // arm up/y motor
    m_elevatorMotor = new CANSparkMax(Constants.ARMELEVATORMOTORID, MotorType.kBrushless);
    // claw go grabby grabby
    m_clawMotor = new CANSparkMax(Constants.ARMCLAWMOTORID, MotorType.kBrushless);
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
