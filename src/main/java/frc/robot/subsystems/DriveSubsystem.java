// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

// import motor & frc dependencies
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;

/** This Subsystem is what allows the code to interact with the drivetrain of the robot. */
public class DriveSubsystem extends SubsystemBase {
  // Gyro Subsystem
  private final GyroSubsystem m_gyroSubsystem;

  // motors
  private final WPI_VictorSPX m_backLeft;
  private final WPI_VictorSPX m_frontLeft;
  private final WPI_VictorSPX m_backRight;
  private final WPI_VictorSPX m_frontRight;
  // Motor Controllers
  private final MotorControllerGroup m_motorsLeft;
  private final MotorControllerGroup m_motorsRight;
  // Main drive function
  private final DifferentialDrive m_ddrive;

  // Encoders
  private final Encoder m_encoderLeft;
  private final Encoder m_encoderRight;

  // Angle PID / RotateToAngle
  static final double turn_P = 0.1;
  static final double turn_I = 0.00;
  static final double turn_D = 0.00;
  static final double MaxTurnRateDegPerS = 100;
  static final double MaxTurnAccelerationDegPerSSquared = 300;
  static final double TurnToleranceDeg = 3; // max diff in degrees
  static final double TurnRateToleranceDegPerS = 10; // degrees per second
  // false when inactive, true when active / a target is set.
  private boolean turnControllerEnabled = false;
  private double turnRotateToAngleRate; // This value will be updated by the PID Controller
  // pid controller for "RotateToAngle"
  private final ProfiledPIDController m_turnController =
      new ProfiledPIDController(
          turn_P,
          turn_I,
          turn_D,
          new TrapezoidProfile.Constraints(MaxTurnRateDegPerS, MaxTurnAccelerationDegPerSSquared));

  // Balance PID / AutoBalance

  static final double balance_P = 0.0625; // 1/16
  static final double balance_I = 0.00;
  static final double balance_D = 0.00;
  static final double MaxBalanceRateDegPerS = 10;
  static final double MaxBalanceAccelerationDegPerSSquared = 20;
  static final double BalanceToleranceDeg = 2; // max diff in degrees
  // false when inactive, true when active / a target is set.
  private boolean balanceControllerEnabled = false;
  private double balanceThrottleRate; // This value will be updated by the PID Controller
  // pid controller for balanceCorrection
  private final ProfiledPIDController m_balanceController =
      new ProfiledPIDController(
          balance_P,
          balance_I,
          balance_D,
          new TrapezoidProfile.Constraints(
              MaxBalanceRateDegPerS, MaxBalanceAccelerationDegPerSSquared));

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_driveOdometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(final GyroSubsystem g_subsystem) {
    m_gyroSubsystem = g_subsystem;
    // init motors
    // rio means built into the roboRIO
    m_backLeft = new WPI_VictorSPX(CANConstants.MOTORBACKLEFTID);
    m_frontLeft = new WPI_VictorSPX(CANConstants.MOTORFRONTLEFTID);
    m_frontRight = new WPI_VictorSPX(CANConstants.MOTORFRONTRIGHTID);
    m_backRight = new WPI_VictorSPX(CANConstants.MOTORBACKRIGHTID);

    // init motor controller groups
    m_motorsLeft = new MotorControllerGroup(m_backLeft, m_frontLeft);
    m_motorsRight = new MotorControllerGroup(m_frontRight, m_backRight);
    m_motorsRight.setInverted(true); // invert left side

    // init drive function
    m_ddrive = new DifferentialDrive(m_motorsLeft, m_motorsRight);
    // config pid controller for motors.
    m_turnController.enableContinuousInput(-180.0f, 180.0f);
    m_turnController.setTolerance(TurnToleranceDeg, TurnRateToleranceDegPerS);
    // this is the target pitch/ tilt error.
    m_balanceController.setGoal(0);
    m_balanceController.setTolerance(BalanceToleranceDeg); // max error in degrees
    // init Encoders
    m_encoderLeft = new Encoder(Constants.DRIVEENCODERLEFTA, Constants.DRIVEENCODERLEFTB);
    m_encoderRight = new Encoder(Constants.DRIVEENCODERRIGHTA, Constants.DRIVEENCODERRIGHTB);
    m_encoderRight.setReverseDirection(true); // invert left to match drive
    // configure encoders
    m_encoderLeft.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE); // distance in meters
    m_encoderRight.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE); // distance in meters
    m_encoderLeft.setSamplesToAverage(5);
    m_encoderRight.setSamplesToAverage(5);
    m_encoderLeft.setMinRate(0.1); // min rate to be determined moving
    m_encoderRight.setMinRate(0.1); // min rate to be determined moving
    resetEncoders();
    m_gyroSubsystem.reset();
    // configure Odemetry
    m_driveOdometry =
        new DifferentialDriveOdometry(
            m_gyroSubsystem.getRotation2d(),
            m_encoderLeft.getDistance(),
            m_encoderRight.getDistance());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_motorsLeft.setVoltage(leftVolts);
    m_motorsRight.setVoltage(rightVolts);
    m_ddrive.feed();
  }

  // default tank drive function
  // **tank drive = specific control style where two parallel forces of motion are controlled to
  // create linear and rotational motion
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_ddrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void balanceResetPID() {
    balanceControllerEnabled = false;
  }

  public void balanceCorrection(double gyroPitchAngle) {
    if (!balanceControllerEnabled) {
      m_balanceController.reset(gyroPitchAngle);
      balanceControllerEnabled = true;
    }
    balanceThrottleRate = MathUtil.clamp(m_balanceController.calculate(gyroPitchAngle), -1.0, 1.0);
    if (!m_balanceController.atGoal()) {
      // we reverse the values beacuse the robot was balancing in the wrong direction.
      this.tankDrive(-balanceThrottleRate, -balanceThrottleRate);
      System.out.println(balanceThrottleRate);
    }
  }
  // these next 4 functions are for turning a set radius while using the gyro.
  public void turnResetPID() {
    /** This should be run when stopping a pid command. */
    turnControllerEnabled = false;
  }

  private void calcuateAngleRate(double gyroYawAngle, double TargetAngleDegrees) {
    if (!turnControllerEnabled) {
      m_turnController.reset(gyroYawAngle);
      m_turnController.setGoal(TargetAngleDegrees);
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
    if (m_turnController.atGoal()) {
      this.tankDrive(leftStickValue, rightStickValue);
    }
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
    if (m_turnController.atGoal()) {
      this.tankDrive(leftStickValue, rightStickValue);
    }
  }

  // for odemetry (path following)
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_encoderLeft.getRate(), m_encoderRight.getRate());
  }

  public Pose2d getPose() {
    return m_driveOdometry.getPoseMeters();
  }

  public void resetEncoders() {
    m_encoderLeft.reset();
    m_encoderRight.reset();
  }

  public double AverageDistance() {
    return (m_encoderLeft.getDistance() + m_encoderRight.getDistance()) / 2;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_driveOdometry.resetPosition(
        m_gyroSubsystem.getRotation2d(),
        m_encoderLeft.getDistance(),
        m_encoderRight.getDistance(),
        pose);
  }

  public void stop() {
    this.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Speed (M/s)", this.m_encoderLeft.getRate());
    SmartDashboard.putNumber("Right Encoder Speed (M/s)", this.m_encoderRight.getRate());
    SmartDashboard.putNumber("Distance L", this.m_encoderLeft.getDistance());
    SmartDashboard.putNumber("Distance R", this.m_encoderRight.getDistance());
    SmartDashboard.putNumber("Current Robot Location X axis", getPose().getX());
    SmartDashboard.putNumber("Current Robot Location Y axis", getPose().getY());
    SmartDashboard.putNumber("Current Robot Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Average Distance Traveled", AverageDistance());
    // Update the odometry in the periodic block
    m_driveOdometry.update(
        m_gyroSubsystem.getRotation2d(), m_encoderLeft.getDistance(), m_encoderRight.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
