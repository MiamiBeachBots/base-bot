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

  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;

  // Variables for Angle PID

  // false when inactive, true when active / a target is set.
  private boolean turnControllerEnabled = false;
  private double rotateToAngleRate; // This value will be updated by the PID Controller

  // pid controller for "RotateToAngle"
  private final PIDController m_turnController = new PIDController(kP, kI, kD);

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
  }

  // default tank drive function
  // **tank drive = specific control style where two parallel forces of motion are controlled to
  // create linear and rotational motion
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_ddrive.tankDrive(
        Constants.MAX_SPEED * leftSpeed,
        -Constants.MAX_SPEED * rightSpeed); // negative as motors are swapped rn
  }

  public void backward() {
    this.tankDrive(0.5, 0.5);
  }

  public void stop() {
    this.tankDrive(0, 0);
  }

  public void resetPID() {
    /** This should be run when stopping a pid command. */
    turnControllerEnabled = false;
  }

  private void calcuateAngleRate(double gyroYawAngle, double TargetAngleDegrees) {
    if (!turnControllerEnabled) {
      m_turnController.setSetpoint(TargetAngleDegrees);
      turnControllerEnabled = true;
    }
    rotateToAngleRate = MathUtil.clamp(m_turnController.calculate(gyroYawAngle), -1.0, 1.0);
  }

  public void turnToAngle(double gyroYawAngle, double TargetAngleDegrees) {
    /*
     * When this function is activated, execute another command to rotate to target angle. Since a Tank drive
     * system cannot move forward simultaneously while rotating, all joystick input
     * is ignored until this button is released.
     */
    this.calcuateAngleRate(gyroYawAngle, TargetAngleDegrees);
    double leftStickValue = rotateToAngleRate;
    double rightStickValue = rotateToAngleRate;
    this.tankDrive(leftStickValue, rightStickValue);
  }

  // magnitude = (joystickL + joystickR) / 2;
  public void driveStraight( // we command you to stay straight,
      double gyroYawAngle, double gyroAccumYawAngle, double joystickMagnitude) {
    /*
     * WWhen this function is activated, the robot is in "drive straight" mode.
     * Whatever direction the robot was heading when "drive straight" mode was
     * entered will be maintained. The average speed of both joysticks is the
     * magnitude of motion.
     */
    this.calcuateAngleRate(gyroYawAngle, gyroAccumYawAngle);
    double leftStickValue = joystickMagnitude + rotateToAngleRate;
    double rightStickValue = joystickMagnitude - rotateToAngleRate;
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
