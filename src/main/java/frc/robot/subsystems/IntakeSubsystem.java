// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private Victor intakeMotorFront;

  private Victor intakeMotorBack;

  public IntakeSubsystem() {
    intakeMotorFront = new Victor(Constants.INTAKEMOTORFRONTCHANNEL);
    intakeMotorBack = new Victor(Constants.INTAKEMOTORBACKCHANNEL);
  }

  public void frontIntake(double liftSpeed) {
    intakeMotorFront.set(liftSpeed);
  }

  public void backIntake(double liftSpeed) {
    intakeMotorBack.set(liftSpeed);
  }

  public void intake(double liftSpeed) {
    intakeMotorBack.set(liftSpeed);
    intakeMotorFront.set(liftSpeed);
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
