// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotorMain, m_armMotorSecondary;
  private final SparkPIDController m_armMainPIDController;
  private final RelativeEncoder m_MainEncoder, m_SecondaryEncoder;
  private final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private final double kLoweredArmPositionRadians = Units.degreesToRadians(45);
  // general drive constants
  // https://www.chiefdelphi.com/t/encoders-velocity-to-m-s/390332/2
  // https://sciencing.com/convert-rpm-linear-speed-8232280.html
  private final double kGearRatio = 1; // TBD
  // basically converted from rotations to radians by multiplying by 2 pi, then adjusting for the
  // gear ratio by dividing by the gear ratio.
  // remember that 2pi radians in 360 degrees.
  private final double kRadiansConversionRatio = (Math.PI * 2) / kGearRatio;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // create the arm motors
    m_armMotorMain = new CANSparkMax(CANConstants.MOTORARMMAINID, CANSparkMax.MotorType.kBrushless);
    m_armMotorSecondary =
        new CANSparkMax(CANConstants.MOTORARMSECONDARYID, CANSparkMax.MotorType.kBrushless);

    // set the idle mode to brake
    m_armMotorMain.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_armMotorSecondary.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // set the secondary motor to follow the main motor
    m_armMotorSecondary.follow(m_armMotorMain);

    // connect to built in PID controller
    m_armMainPIDController = m_armMotorMain.getPIDController();
    // allow us to read the encoder
    m_MainEncoder = m_armMotorMain.getEncoder();
    m_SecondaryEncoder = m_armMotorSecondary.getEncoder();
    // setup the encoders
    m_MainEncoder.setPositionConversionFactor(kRadiansConversionRatio);
    m_SecondaryEncoder.setPositionConversionFactor(kRadiansConversionRatio);
    // PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    m_armMainPIDController.setP(kP);
    m_armMainPIDController.setI(kI);
    m_armMainPIDController.setD(kD);
    m_armMainPIDController.setIZone(kIz);
    m_armMainPIDController.setFF(kFF);
    m_armMainPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public double getAverageEncoderPosition() {
    // get the average encoder position
    return (m_MainEncoder.getPosition() + m_SecondaryEncoder.getPosition()) / 2;
  }

  /*
   * Move arm a certain number of radians relative to current position
   */
  public void MoveArmRelative(double radians) {
    radians = radians + getAverageEncoderPosition();
    // update the PID controller with current encoder position
    MoveArmToPosition(radians);
  }

  public void lowerArm() {
    // move to set lowered arm position
    MoveArmToPosition(kLoweredArmPositionRadians);
  }

  /*
   * Move arm to global position
   */
  public void MoveArmToPosition(double radians) {
    // update the PID controller with current encoder position
    m_armMainPIDController.setReference(radians, CANSparkBase.ControlType.kPosition);
  }

  /*
   * attempt to hold arm at current location
   */
  public void stop() {
    // update the PID controller with current encoder position
    MoveArmToPosition(getAverageEncoderPosition());
  }

  public void zeroEncoders() {
    m_MainEncoder.setPosition(0);
    m_SecondaryEncoder.setPosition(0);
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
