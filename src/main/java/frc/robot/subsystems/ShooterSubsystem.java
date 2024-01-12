// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_ShooterMotorMain;
  private final SparkPIDController m_ShooterMainPIDController;
  private RelativeEncoder m_ShooterMainEncoder;
  private final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // create the shooter motors
    m_ShooterMotorMain =
        new CANSparkMax(CANConstants.MOTORSHOOTERID, CANSparkMax.MotorType.kBrushless);
    // set the idle mode to coast
    m_ShooterMotorMain.setIdleMode(CANSparkMax.IdleMode.kCoast);

    // connect to built in PID controller
    m_ShooterMainPIDController = m_ShooterMotorMain.getPIDController();

    // allow us to read the encoder
    m_ShooterMainEncoder = m_ShooterMotorMain.getEncoder();
    // PID coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_ShooterMainPIDController.setP(kP);
    m_ShooterMainPIDController.setI(kI);
    m_ShooterMainPIDController.setD(kD);
    m_ShooterMainPIDController.setIZone(kIz);
    m_ShooterMainPIDController.setFF(kFF);
    m_ShooterMainPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /*
   * Spin shooter at a given RPM
   */
  public void SpinShooter(double RPM) {
    m_ShooterMainPIDController.setReference(RPM, CANSparkBase.ControlType.kVelocity);
  }

  /*
   * Stop the shooter
   */
  public void StopShooter() {
    SpinShooter(0);
  }

  /*
   * Spin Shooter at max RPM
   */
  public void SpinShooterFull() {
    SpinShooter(maxRPM);
  }

  /*
   * Check if shooter is at a given RPM
   */
  public Boolean isAtRPMTolerance(double RPM) {
    return (m_ShooterMainEncoder.getVelocity() > RPM - 100
        && m_ShooterMainEncoder.getVelocity() < RPM + 100);
  }

  /*
   * Check if shooter is at max RPM
   */
  public Boolean isAtMaxRPM() {
    return isAtRPMTolerance(maxRPM);
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
