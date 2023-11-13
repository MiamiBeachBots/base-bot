// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  // Gyro

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

  // Odometry class for tracking robot pose (position on field)
  private final DifferentialDriveOdometry m_driveOdometry;

  private boolean turnControllerEnabled = false;
  private double turnRotateToAngleRate; // This value will be updated by the PID Controller
  // pid controller for "RotateToAngle"
  private final ProfiledPIDController m_turnController =
      new ProfiledPIDController(
          frc.robot.PIDConstants.turnPID.turn_P,
          frc.robot.PIDConstants.turnPID.turn_I,
          frc.robot.PIDConstants.turnPID.turn_D,
          new TrapezoidProfile.Constraints(
              frc.robot.PIDConstants.turnPID.MaxTurnRateDegPerS,
              frc.robot.PIDConstants.turnPID.MaxTurnAccelerationDegPerSSquared));

  // false when inactive, true when active / a target is set.
  private boolean distanceControllerEnabled = false;
  private double distanceThrottleRate; // This value will be updated by the PID Controller
  // pid controller for "MoveDistance"
  private final ProfiledPIDController m_distanceController =
      new ProfiledPIDController(
          frc.robot.PIDConstants.distancePID.distance_P,
          frc.robot.PIDConstants.distancePID.distance_I,
          frc.robot.PIDConstants.distancePID.distance_D,
          new TrapezoidProfile.Constraints(
              frc.robot.PIDConstants.distancePID.distanceMaxSpeed,
              frc.robot.PIDConstants.distancePID.distanceMaxAcceleration));

  // false when inactive, true when active / a target is set.
  private boolean balanceControllerEnabled = false;
  private double balanceThrottleRate; // This value will be updated by the PID Controller
  // pid controller for balanceCorrection
  private final ProfiledPIDController m_balanceController =
      new ProfiledPIDController(
          frc.robot.PIDConstants.balancePID.balance_P,
          frc.robot.PIDConstants.balancePID.balance_I,
          frc.robot.PIDConstants.balancePID.balance_D,
          new TrapezoidProfile.Constraints(
              frc.robot.PIDConstants.balancePID.MaxBalanceRateDegPerS,
              frc.robot.PIDConstants.balancePID.MaxBalanceAccelerationDegPerSSquared));

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
    m_motorsRight.setInverted(true); // invert left side

    // init drive function
    m_ddrive = new DifferentialDrive(m_motorsLeft, m_motorsRight);

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
    // Every time this function is called, A dollar is taken out of Jack's savings. Aka do it more.

    // configure Odemetry
    m_driveOdometry =
        new DifferentialDriveOdometry(
            frc.robot.subsystems.GyroSubsystem.Rotation2d,
            m_encoderLeft.getDistance(),
            m_encoderRight.getDistance());

    // config turn pid controller.
    m_turnController.enableContinuousInput(-180.0f, 180.0f);
    m_turnController.setTolerance(
        frc.robot.PIDConstants.turnPID.TurnToleranceDeg,
        frc.robot.PIDConstants.turnPID.TurnRateToleranceDegPerS);
    // config distance pid controller
    m_distanceController.setTolerance(
        frc.robot.PIDConstants.distancePID.DistanceTolerance,
        frc.robot.PIDConstants.distancePID.DistanceSpeedTolerance);
    // this is the target pitch/ tilt error.
    m_balanceController.setGoal(0);
    m_balanceController.setTolerance(
        frc.robot.PIDConstants.balancePID.BalanceToleranceDeg); // max error in degrees
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
    /** This should be run when stopping a pid command. */
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

  public void distanceResetPID() {
    /** This should be run when stopping a pid command. */
    distanceControllerEnabled = false;
  }

  public void distanceSetGoal(double targetDistance) {
    m_distanceController.setGoal(AverageDistance() + targetDistance);
  }

  private void calculateDistanceRate(double targetDistance) {
    if (!distanceControllerEnabled) {
      m_distanceController.reset(AverageDistance());
      m_distanceController.setGoal(AverageDistance() + targetDistance);
      distanceControllerEnabled = true;
    }
    distanceThrottleRate =
        MathUtil.clamp(m_distanceController.calculate(AverageDistance()), -1.0, 1.0);
  }

  public void driveToDistance(double targetDistance) {
    this.calculateDistanceRate(targetDistance);
    double leftStickValue = turnRotateToAngleRate;
    double rightStickValue = turnRotateToAngleRate;
    if (!m_distanceController.atGoal()) {
      this.tankDrive(leftStickValue, rightStickValue);
    }
  }

  public void driveAndTurn(double gyroYawAngle, double TargetAngleDegrees, double targetDistance) {
    /*
    This lets you set a gyro angle and a distance you need to travel.
    this should not be used in auto mode.
     */
    this.calcuateAngleRate(gyroYawAngle, TargetAngleDegrees);
    this.calculateDistanceRate(targetDistance);
    double leftStickValue = distanceThrottleRate + turnRotateToAngleRate;
    double rightStickValue = distanceThrottleRate - turnRotateToAngleRate;
    if (!m_distanceController.atGoal() || !m_turnController.atGoal()) {
      this.tankDrive(leftStickValue, rightStickValue);
    }
  }

  // these next 4 functions are for turning a set radius while using the gyro.
  public void turnResetPID() {
    /** This should be run when stopping a pid command. */
    turnControllerEnabled = false;
  }

  public void turnSetGoal(double targetAngleDegrees) {
    m_turnController.setGoal(targetAngleDegrees);
  }

  private void calcuateAngleRate(double gyroYawAngle, double targetAngleDegrees) {
    if (!turnControllerEnabled) {
      m_turnController.reset(gyroYawAngle);
      m_turnController.setGoal(targetAngleDegrees);
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
    double rightStickValue = -turnRotateToAngleRate;
    if (!m_turnController.atGoal()) {
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
    double angleRate;
    if (!m_turnController.atGoal()) {
      angleRate = turnRotateToAngleRate;
    } else {
      angleRate = 0;
    }
    double leftStickValue = joystickMagnitude + angleRate;
    double rightStickValue = joystickMagnitude - angleRate;
    this.tankDrive(leftStickValue, rightStickValue);
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

  public void SetBrakemode() {
    m_backLeft.setNeutralMode(NeutralMode.Brake);
    m_backRight.setNeutralMode(NeutralMode.Brake);
    m_frontLeft.setNeutralMode(NeutralMode.Brake);
    m_frontRight.setNeutralMode(NeutralMode.Brake);
  }

  public void SetCoastmode() {
    m_backLeft.setNeutralMode(NeutralMode.Coast);
    m_backRight.setNeutralMode(NeutralMode.Coast);
    m_frontLeft.setNeutralMode(NeutralMode.Coast);
    m_frontRight.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_driveOdometry.resetPosition(
        frc.robot.subsystems.GyroSubsystem.Rotation2d,
        m_encoderLeft.getDistance(),
        m_encoderRight.getDistance(),
        pose);
  }

  public void stop() {
    this.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Left Encoder Speed (M/s)", this.m_encoderLeft.getRate());
    SmartDashboard.putNumber("Right Encoder Speed (M/s)", this.m_encoderRight.getRate());
    SmartDashboard.putNumber("Distance L", m_encoderLeft.getDistance());
    SmartDashboard.putNumber("Distance R", this.m_encoderRight.getDistance());
    SmartDashboard.putNumber("Current Robot Location X axis", getPose().getX());
    SmartDashboard.putNumber("Current Robot Location Y axis", getPose().getY());
    SmartDashboard.putNumber("Current Robot Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Average Distance Traveled", AverageDistance());
    m_driveOdometry.update(
        frc.robot.subsystems.GyroSubsystem.Rotation2d,
        m_encoderLeft.getDistance(),
        m_encoderRight.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
