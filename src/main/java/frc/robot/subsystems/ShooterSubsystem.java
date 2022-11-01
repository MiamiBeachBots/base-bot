// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private PWMSparkMax shooterMotor1;

  private PWMSparkMax shooterMotor2;
  private Servo shooterServo;

  public ShooterSubsystem() {
    shooterMotor1 = new PWMSparkMax(Constants.SHOOTERMOTOR1CHANNEL);
    shooterMotor2 = new PWMSparkMax(Constants.SHOOTERMOTOR2CHANNEL);
    shooterServo = new Servo(Constants.SHOOTERSERVOCHANNEL);
  }

  public void shoot(double shotSpeed) {
    shooterMotor1.set(-shotSpeed);
    shooterMotor2.set(shotSpeed);
  }

  public void lift(double position) {
    shooterServo.set(position);
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
