// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

// import motor & frc dependencies
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** This Subsystem is what allows the code to interact with the drivetrain of the robot. */
public class DriveSubsystem extends SubsystemBase {
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
  private final MotorControllerGroup m_left;
  private final MotorControllerGroup m_right;

  // drive function
  private final DifferentialDrive m_ddrive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // init motors
    // rio means built into the roboRIO
    m_backLeft = new WPI_VictorSPX(Constants.MOTORBACKLEFTID);
    m_frontLeft = new WPI_VictorSPX(Constants.MOTORFRONTLEFTID);
    m_frontRight = new WPI_VictorSPX(Constants.MOTORFRONTRIGHTID);
    m_backRight = new WPI_VictorSPX(Constants.MOTORBACKRIGHTID);

    // init motors controllers
    m_left = new MotorControllerGroup(m_backLeft, m_frontLeft);
    m_right = new MotorControllerGroup(m_frontRight, m_backRight);

    // init drive function
    m_ddrive = new DifferentialDrive(m_left, m_right);
    // config pid controller for motors.
    m_turnController.enableContinuousInput(-180.0f, 180.0f);
    // this is the minimum pitch error, it can be from 0-2 degrees (the target)
    m_balanceController.setSetpoint(0);
  }

  // default tank drive function
  // **tank drive = specific control style where two parallel forces of motion are controlled to
  // create linear and rotational motion
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_ddrive.tankDrive(leftSpeed, -rightSpeed); // negative as motors are swapped rn
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
