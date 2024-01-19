// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

// import motor & frc dependencies
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;

/** This Subsystem is what allows the code to interact with the drivetrain of the robot. */
public class DriveSubsystem extends SubsystemBase {
  // Gyro
  private final AHRS m_Gyro;

  // motors
  private final CANSparkMax m_backLeft; // Main / Master Motor for Left
  private final CANSparkMax m_frontLeft; // Slave Motor for Left (Follow Master)
  private final CANSparkMax m_backRight; // Main / Master Motor for Right
  private final CANSparkMax m_frontRight; // Slave Motor for Right (Follow Master)
  // Main drive function
  private final DifferentialDrive m_ddrive;

  // Encoders
  private final RelativeEncoder m_encoderBackLeft;
  private final RelativeEncoder m_encoderFrontLeft;
  private final RelativeEncoder m_encoderBackRight;
  private final RelativeEncoder m_encoderFrontRight;

  // Motor PID Controllers
  private final SparkPIDController m_backLeftPIDController;
  private final SparkPIDController m_backRightPIDController;

  // ex:
  // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Read%20Encoder%20Values/src/main/java/frc/robot/Robot.java

  // Odometry class for tracking robot pose (position on field)
  private final DifferentialDrivePoseEstimator m_driveOdometry;

  // Angle PID / RotateToAngle
  static final double turn_P = 0.1;
  static final double turn_I = 0.00;
  static final double turn_D = 0.00;
  static final double MaxTurnRateDegPerS = 20;
  static final double MaxTurnAccelerationDegPerSSquared = 50;
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

  // track robot field location for dashboard
  private Field2d field = new Field2d();

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Init gyro
    m_Gyro = new AHRS(SPI.Port.kMXP);
    // init motors
    // rio means built into the roboRIO
    m_backLeft = new CANSparkMax(CANConstants.MOTORBACKLEFTID, CANSparkMax.MotorType.kBrushless);
    m_frontLeft = new CANSparkMax(CANConstants.MOTORFRONTLEFTID, CANSparkMax.MotorType.kBrushless);
    m_frontRight =
        new CANSparkMax(CANConstants.MOTORFRONTRIGHTID, CANSparkMax.MotorType.kBrushless);
    m_backRight = new CANSparkMax(CANConstants.MOTORBACKRIGHTID, CANSparkMax.MotorType.kBrushless);

    // invert right side
    m_backRight.setInverted(true);
    m_frontRight.setInverted(true);

    // setup main and secondary motors
    m_frontLeft.follow(m_backLeft); // set front left to follow back left
    m_frontRight.follow(m_backRight); // set front right to follow back right

    // init drive function
    m_ddrive = new DifferentialDrive(m_backLeft, m_backRight);

    // init Encoders, we use all 4 encoders even though only 2 are used in feedback to decrease
    // error
    m_encoderBackLeft = m_backLeft.getEncoder();
    m_encoderFrontLeft = m_frontLeft.getEncoder();
    m_encoderBackRight = m_backRight.getEncoder();
    m_encoderFrontRight = m_frontRight.getEncoder();
    // Encoders inverted with motors

    // init PID Controllers
    m_backLeftPIDController = m_backLeft.getPIDController();
    m_backRightPIDController = m_backRight.getPIDController();

    // configure encoders
    // RPM TO m/s
    m_encoderBackLeft.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    m_encoderBackRight.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    m_encoderFrontLeft.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    m_encoderFrontRight.setVelocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    // rotations to meters
    m_encoderBackLeft.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    m_encoderBackRight.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    m_encoderFrontLeft.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    m_encoderFrontRight.setPositionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    resetEncoders();
    // Every time this function is called, A dollar is taken out of Jack's savings. Aka do it more.
    resetGyro();

    // setup PID controllers
    configureMotorPIDControllers();

    // Configure RIO PID Controllers
    // config turn pid controller.
    m_turnController.enableContinuousInput(-180.0f, 180.0f);
    m_turnController.setTolerance(TurnToleranceDeg, TurnRateToleranceDegPerS);
    // this is the target pitch/ tilt error.
    m_balanceController.setGoal(0);
    m_balanceController.setTolerance(BalanceToleranceDeg); // max error in degrees

    // configure Odemetry
    m_driveOdometry =
        new DifferentialDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getRotation2d(),
            getPositionLeft(),
            getPositionRight(),
            new Pose2d());

    // Setup Base AutoBuilder (Autonomous)
    AutoBuilder.configureRamsete(
        this::getPose, // Pose2d supplier
        this::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        this::getSpeeds, // A method for getting the chassis' current speed and direction
        this::setSpeeds, // A consumer that takes the desired chassis speed and direction
        DriveConstants.autoReplanningConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

    SmartDashboard.putData("Field", field); // add field to dashboard

    // setup SysID for auto profiling
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage, voltage),
                null, // No log consumer, since data is recorded by URCL
                this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  private void configureMotorPIDControllers() {
    // setup velocity PID controllers (used by auto)
    m_backLeftPIDController.setP(
        DriveConstants.kPDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightPIDController.setP(
        DriveConstants.kPDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backLeftPIDController.setI(
        DriveConstants.kIDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightPIDController.setI(
        DriveConstants.kIDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backLeftPIDController.setD(
        DriveConstants.kDDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightPIDController.setD(
        DriveConstants.kDDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backLeftPIDController.setIZone(
        DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightPIDController.setIZone(
        DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backLeftPIDController.setFF(
        DriveConstants.kDriveFeedForward, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightPIDController.setFF(
        DriveConstants.kDriveFeedForward, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backLeftPIDController.setOutputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightPIDController.setOutputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainVelocityPIDSlot);

    // setup position PID controllers (used when we manually path find)
    m_backLeftPIDController.setP(
        DriveConstants.kPDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setP(
        DriveConstants.kPDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backLeftPIDController.setI(
        DriveConstants.kIDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setI(
        DriveConstants.kIDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backLeftPIDController.setD(
        DriveConstants.kDDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setD(
        DriveConstants.kDDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backLeftPIDController.setIZone(
        DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setIZone(
        DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backLeftPIDController.setFF(
        DriveConstants.kDriveFeedForward, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setFF(
        DriveConstants.kDriveFeedForward, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backLeftPIDController.setOutputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setOutputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainPositionPIDSlot);
  }

  public void setVoltage(Measure<Voltage> rightVoltage, Measure<Voltage> leftVoltage) {
    m_backLeft.setVoltage(leftVoltage.in(Volts));
    m_backRight.setVoltage(rightVoltage.in(Volts));
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

  public void driveToRelativePosition(double targetPosition) {
    if (targetPosition < 0.1) { // less then 0.1 meters, do nothing, pervents feedback loop
      double totalPosition = this.AverageDistance() + targetPosition;
      this.driveToPosition(totalPosition);
    }
  }

  public void driveAndTurn(double gyroYawAngle, double TargetAngleDegrees, double targetDistance) {
    /*
    This lets you set a gyro angle and a distance you need to travel.
    this should not be used in auto mode.
     */
    this.calcuateAngleRate(gyroYawAngle, TargetAngleDegrees);
    double leftStickValue = turnRotateToAngleRate;
    double rightStickValue = turnRotateToAngleRate;
    if (!m_turnController.atGoal()) {
      this.tankDrive(leftStickValue, rightStickValue);
    } else {
      this.driveToRelativePosition(targetDistance);
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

  /*
   * This function can return our robots DiffernentialDriveWheelSpeeds, which is the speed of each side of the robot.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
  }

  /*
   * This function can return our robots ChassisSpeeds, which is vx (m/s), vy (m/s), and omega (rad/s).
   */
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /*
   * This function can set our robots ChassisSpeeds, which is vx (m/s), vy (m/s), and omega (rad/s).
   * vy is always 0 as we are not strafing.
   */
  public void setSpeeds(ChassisSpeeds speeds) {
    setWheelVelocities(DriveConstants.kDriveKinematics.toWheelSpeeds(speeds));
  }

  /*
   * This function can set our robots DifferentialDriveWheelSpeeds, which is the speed of each side of the robot.
   */
  public void setWheelVelocities(DifferentialDriveWheelSpeeds speeds) {
    // get left and right speeds in m/s
    double leftSpeed = speeds.leftMetersPerSecond;
    double rightSpeed = speeds.rightMetersPerSecond;
    // set to position of motors
    m_backLeftPIDController.setReference(
        leftSpeed, CANSparkBase.ControlType.kVelocity, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightPIDController.setReference(
        rightSpeed, CANSparkBase.ControlType.kVelocity, DriveConstants.kDrivetrainVelocityPIDSlot);
  }

  // in meters, use averageDistance() to get average distance traveled, as an offset to set this
  // function.
  public void driveToPosition(double NewPosition) {
    m_backLeftPIDController.setReference(
        NewPosition, CANSparkBase.ControlType.kPosition, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setReference(
        NewPosition, CANSparkBase.ControlType.kPosition, DriveConstants.kDrivetrainPositionPIDSlot);
  }

  public Pose2d getPose() {
    return m_driveOdometry.getEstimatedPosition();
  }

  public void updateVisionPose(Pose2d visionRobotPose, double timestamp) {
    m_driveOdometry.addVisionMeasurement(visionRobotPose, timestamp);
  }

  public void resetEncoders() {
    m_encoderBackLeft.setPosition(0);
    m_encoderFrontLeft.setPosition(0);
    m_encoderBackRight.setPosition(0);
    m_encoderFrontRight.setPosition(0);
  }

  public double AverageDistance() {
    return (getPositionLeft() + getPositionRight()) / 2;
  }

  public void SetBrakemode() {
    m_backLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_backRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void SetCoastmode() {
    m_backLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_backRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_frontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_frontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_driveOdometry.resetPosition(getRotation2d(), getPositionLeft(), getPositionRight(), pose);
  }

  public void stop() {
    this.tankDrive(0, 0);
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

  public double getVelocityLeft() {
    return (m_encoderBackLeft.getVelocity() + m_encoderFrontLeft.getVelocity()) / 2;
  }

  public double getVelocityRight() {
    return (m_encoderBackRight.getVelocity() + m_encoderFrontRight.getVelocity()) / 2;
  }

  public double getPositionLeft() {
    return (m_encoderBackLeft.getPosition() + m_encoderFrontLeft.getPosition()) / 2;
  }

  public double getPositionRight() {
    return (m_encoderBackRight.getPosition() + m_encoderFrontRight.getPosition()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Speed (M/s)", this.getVelocityLeft());
    SmartDashboard.putNumber("Right Encoder Speed (M/s)", this.getVelocityRight());
    SmartDashboard.putNumber("Distance L", this.getPositionLeft());
    SmartDashboard.putNumber("Distance R", this.getPositionRight());
    SmartDashboard.putNumber("Current Robot Location X axis", getPose().getX());
    SmartDashboard.putNumber("Current Robot Location Y axis", getPose().getY());
    SmartDashboard.putNumber("Current Robot Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Average Distance Traveled", AverageDistance());
    SmartDashboard.putNumber("Current Gyro Pitch", getPitch());
    SmartDashboard.putNumber("Current Gyro Yaw", getYaw());
    SmartDashboard.putBoolean("Gyro Calibrating", m_Gyro.isCalibrating());
    // Update the odometry in the periodic block
    m_driveOdometry.update(getRotation2d(), getPositionLeft(), getPositionRight());
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
