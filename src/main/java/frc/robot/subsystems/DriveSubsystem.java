// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

// import motor & frc dependencies
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;
import frc.robot.Constants;

/** This Subsystem is what allows the code to interact with the drivetrain of the robot. */
public class DriveSubsystem extends SubsystemBase {
  // Drive train characteristics
  private static final double wheelDiameter = 6; // inches
  private static final double pulsesPerRevolution = 2048; // resolution of encoder
  private static final double GEAR_RATIO = 1;
  private static final double distancePerPulse =
      (Math.PI * wheelDiameter) / pulsesPerRevolution / GEAR_RATIO;
  // constants for Angle PID

  static final double turn_P = 0.03;
  static final double turn_I = 0.00;
  static final double turn_D = 0.00;

  // Variables for Angle PID

  // false when inactive, true when active / a target is set.
  private boolean turnControllerEnabled = false;
  private double turnRotateToAngleRate; // This value will be updated by the PID Controller

  // pid controller for "RotateToAngle"
  private final PIDController m_turnController = new PIDController(turn_P, turn_I, turn_D);

  // constants for Balance PID

  static final double balance_P = 0.0625; // 1/16
  static final double balance_I = 0.00;
  static final double balance_D = 0.00;

  // Variables for Balance PID

  private double balanceThrottleRate; // This value will be updated by the PID Controller

  // pid controller for balanceCorrection
  private final PIDController m_balanceController =
      new PIDController(balance_P, balance_I, balance_D);

  // motors
  private final WPI_VictorSPX m_backLeft;
  private final WPI_VictorSPX m_frontLeft;
  private final WPI_VictorSPX m_backRight;
  private final WPI_VictorSPX m_frontRight;

  // motors controllers
  private final MotorControllerGroup m_motorsLeft;
  private final MotorControllerGroup m_motorsRight;

  // drive function
  private final DifferentialDrive m_ddrive;

  // encoders
  private final Encoder m_encoderLeft;
  private final Encoder m_encoderRight;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // init motors
    // rio means built into the roboRIO
    m_backLeft = new WPI_VictorSPX(CANConstants.MOTORBACKLEFTID);
    m_frontLeft = new WPI_VictorSPX(CANConstants.MOTORFRONTLEFTID);
    m_frontRight = new WPI_VictorSPX(CANConstants.MOTORFRONTRIGHTID);
    m_backRight = new WPI_VictorSPX(CANConstants.MOTORBACKRIGHTID);

    // init motor controller groups
    m_motorsLeft = new MotorControllerGroup(m_backLeft, m_frontLeft);
    m_motorsRight = new MotorControllerGroup(m_frontRight, m_backRight);
    m_motorsRight.setInverted(true); // invert right side

    // init drive function
    m_ddrive = new DifferentialDrive(m_motorsLeft, m_motorsRight);
    // config pid controller for motors.
    m_turnController.enableContinuousInput(-180.0f, 180.0f);
    // this is the target pitch/ tilt error.
    m_balanceController.setSetpoint(0);
    // init Encoders
    m_encoderLeft = new Encoder(Constants.DRIVEENCODERLEFTA, Constants.DRIVEENCODERLEFTB);
    m_encoderRight = new Encoder(Constants.DRIVEENCODERRIGHTA, Constants.DRIVEENCODERRIGHTB);
    m_encoderRight.setReverseDirection(true);
    // configure encoders
    m_encoderLeft.setDistancePerPulse(distancePerPulse); // distance in inches
    m_encoderRight.setDistancePerPulse(distancePerPulse); // distance in inches
    m_encoderLeft.setSamplesToAverage(5);
    m_encoderRight.setSamplesToAverage(5);
    m_encoderLeft.setMinRate(6); // min rate to be determined moving
    m_encoderRight.setMinRate(6); // min rate to be determined moving
    m_encoderLeft.reset(); // clear encoder
    m_encoderRight.reset(); // clear encoder
  }

  // default tank drive function
  // **tank drive = specific control style where two parallel forces of motion are controlled to
  // create linear and rotational motion
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_ddrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stop() {
    this.tankDrive(0, 0);
  }

  public void balanceCorrection(double gyroPitchAngle) {
    balanceThrottleRate = MathUtil.clamp(m_turnController.calculate(gyroPitchAngle), -1.0, 1.0);
    System.out.println(balanceThrottleRate);
    this.tankDrive(balanceThrottleRate, balanceThrottleRate);
  }
  // these next 4 functions are for turning a set radius while using the gyro.
  public void turnResetPID() {
    /** This should be run when stopping a pid command. */
    turnControllerEnabled = false;
  }

  private void calcuateAngleRate(double gyroYawAngle, double TargetAngleDegrees) {
    if (!turnControllerEnabled) {
      m_turnController.setSetpoint(TargetAngleDegrees);
      turnControllerEnabled = true;
    }
    turnRotateToAngleRate = MathUtil.clamp(m_turnController.calculate(gyroYawAngle), -1.0, 1.0);
  }

  public void turnToAngle(double gyroYawAngle, double TargetAngleDegrees) {
    /*
     * When this function is activated, execute another command to rotate to target angle. Since a Tank drive
     * system cannot move forward simultaneously while rotating, all joystick input
     * is ignored until this button is released.
     */
    this.calcuateAngleRate(gyroYawAngle, TargetAngleDegrees);
    double leftStickValue = turnRotateToAngleRate;
    double rightStickValue = turnRotateToAngleRate;
    this.tankDrive(leftStickValue, rightStickValue);
  }

  // magnitude = (joystickL + joystickR) / 2;
  public void driveStraight(
      double gyroYawAngle, double gyroAccumYawAngle, double joystickMagnitude) {
    /*
     * WWhen this function is activated, the robot is in "drive straight" mode.
     * Whatever direction the robot was heading when "drive straight" mode was
     * entered will be maintained. The average speed of both joysticks is the
     * magnitude of motion.
     */
    this.calcuateAngleRate(gyroYawAngle, gyroAccumYawAngle);
    double leftStickValue = joystickMagnitude + turnRotateToAngleRate;
    double rightStickValue = joystickMagnitude - turnRotateToAngleRate;
    this.tankDrive(leftStickValue, rightStickValue);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Speed (In/s)", this.m_encoderLeft.getRate());
    SmartDashboard.putNumber("Right Encoder Speed (In/s)", this.m_encoderRight.getRate());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
