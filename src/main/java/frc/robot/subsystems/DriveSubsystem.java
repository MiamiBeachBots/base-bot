// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// import motor & frc dependencies
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.DriveConstants;
import frc.robot.PIDControllers;

/** This Subsystem is what allows the code to interact with the drivetrain of the robot. */
public class DriveSubsystem extends SubsystemBase {
  // Gyro
  private final AHRS m_Gyro;

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

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Init gyro
    m_Gyro = new AHRS(SPI.Port.kMXP);
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
    resetGyro();

    // configure Odemetry
    m_driveOdometry =
        new DifferentialDriveOdometry(
            getRotation2d(), m_encoderLeft.getDistance(), m_encoderRight.getDistance());

    // config turn pid controller.
    PIDControllers.m_turnController.enableContinuousInput(-180.0f, 180.0f);
    PIDControllers.m_turnController.setTolerance(
        PIDConstants.TurnPIDConstants.TurnToleranceDeg,
        PIDConstants.TurnPIDConstants.TurnRateToleranceDegPerS);
    // config distance pid controller
    PIDControllers.m_distanceController.setTolerance(
        PIDConstants.DistancePIDConstants.DistanceTolerance,
        PIDConstants.DistancePIDConstants.DistanceSpeedTolerance);
    // this is the target pitch/ tilt error.
    PIDControllers.m_balanceController.setGoal(0);
    PIDControllers.m_balanceController.setTolerance(
        PIDConstants.BalancePIDConstants.BalanceToleranceDeg); // max error in degrees
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
    PIDControllers.balanceControllerEnabled = false;
  }

  public void balanceCorrection(double gyroPitchAngle) {
    if (!PIDControllers.balanceControllerEnabled) {
      PIDControllers.m_balanceController.reset(gyroPitchAngle);
      PIDControllers.balanceControllerEnabled = true;
    }
    PIDControllers.balanceThrottleRate =
        MathUtil.clamp(PIDControllers.m_balanceController.calculate(gyroPitchAngle), -1.0, 1.0);
    if (!PIDControllers.m_balanceController.atGoal()) {
      // we reverse the values beacuse the robot was balancing in the wrong direction.
      this.tankDrive(-PIDControllers.balanceThrottleRate, -PIDControllers.balanceThrottleRate);
      System.out.println(PIDControllers.balanceThrottleRate);
    }
  }

  public void distanceResetPID() {
    /** This should be run when stopping a pid command. */
    PIDControllers.distanceControllerEnabled = false;
  }

  public void distanceSetGoal(double targetDistance) {
    PIDControllers.m_distanceController.setGoal(AverageDistance() + targetDistance);
  }

  private void calculateDistanceRate(double targetDistance) {
    if (!PIDControllers.distanceControllerEnabled) {
      PIDControllers.m_distanceController.reset(AverageDistance());
      PIDControllers.m_distanceController.setGoal(AverageDistance() + targetDistance);
      PIDControllers.distanceControllerEnabled = true;
    }
    PIDControllers.distanceThrottleRate =
        MathUtil.clamp(PIDControllers.m_distanceController.calculate(AverageDistance()), -1.0, 1.0);
  }

  public void driveToDistance(double targetDistance) {
    this.calculateDistanceRate(targetDistance);
    double leftStickValue = PIDControllers.turnRotateToAngleRate;
    double rightStickValue = PIDControllers.turnRotateToAngleRate;
    if (!PIDControllers.m_distanceController.atGoal()) {
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
    double leftStickValue =
        PIDControllers.distanceThrottleRate + PIDControllers.turnRotateToAngleRate;
    double rightStickValue =
        PIDControllers.distanceThrottleRate - PIDControllers.turnRotateToAngleRate;
    if (!PIDControllers.m_distanceController.atGoal()
        || !PIDControllers.m_turnController.atGoal()) {
      this.tankDrive(leftStickValue, rightStickValue);
    }
  }

  // these next 4 functions are for turning a set radius while using the gyro.
  public void turnResetPID() {
    /** This should be run when stopping a pid command. */
    PIDControllers.turnControllerEnabled = false;
  }

  public void turnSetGoal(double targetAngleDegrees) {
    PIDControllers.m_turnController.setGoal(targetAngleDegrees);
  }

  private void calcuateAngleRate(double gyroYawAngle, double targetAngleDegrees) {
    if (!PIDControllers.turnControllerEnabled) {
      PIDControllers.m_turnController.reset(gyroYawAngle);
      PIDControllers.m_turnController.setGoal(targetAngleDegrees);
      PIDControllers.turnControllerEnabled = true;
    }
    PIDControllers.turnRotateToAngleRate =
        MathUtil.clamp(PIDControllers.m_turnController.calculate(gyroYawAngle), -1.0, 1.0);
  }

  public void turnToAngle(double gyroYawAngle, double TargetAngleDegrees) {
    /*
     * When this function is activated, execute another command to rotate to target angle. Since a Tank drive
     * system cannot move forward simultaneously while rotating, all joystick input
     * is ignored until this button is released.
     */
    this.calcuateAngleRate(gyroYawAngle, TargetAngleDegrees);
    double leftStickValue = PIDControllers.turnRotateToAngleRate;
    double rightStickValue = -PIDControllers.turnRotateToAngleRate;
    if (!PIDControllers.m_turnController.atGoal()) {
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
    if (!PIDControllers.m_turnController.atGoal()) {
      angleRate = PIDControllers.turnRotateToAngleRate;
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
        getRotation2d(), m_encoderLeft.getDistance(), m_encoderRight.getDistance(), pose);
  }

  public void stop() {
    this.tankDrive(0, 0);
  }

  public void calibrate() {
    m_Gyro.calibrate();
  }

  public Rotation2d getRotation2d() {
    return m_Gyro.getRotation2d();
  }

  // for balance correction
  public double getPitch() {
    return m_Gyro.getPitch(); // get pitch in degrees
  }

  // for PID control (turn by degrees)
  public double getAccumYaw() {
    return m_Gyro.getAngle(); // get angle in degrees
  }

  public double getYaw() {
    return m_Gyro.getYaw();
  }

  public void resetGyro() {
    m_Gyro.reset();
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
    SmartDashboard.putNumber("Current Gyro Pitch", getPitch());
    SmartDashboard.putNumber("Current Gyro Yaw", getYaw());
    // Update the odometry in the periodic block
    m_driveOdometry.update(
        getRotation2d(), m_encoderLeft.getDistance(), m_encoderRight.getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
