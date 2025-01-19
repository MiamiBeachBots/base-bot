// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

// import motor & frc dependencies
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;
import frc.robot.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** This Subsystem is what allows the code to interact with the drivetrain of the robot. */
public class DriveSubsystem extends SubsystemBase {
  // Gyro
  private final AHRS m_Gyro;

  // simulation gyro
  private final SimDouble SimGyroAngleHandler;

  // motors
  private final SparkMax m_backLeft; // Main / Master Motor for Left
  private final SparkMax m_frontLeft; // Slave Motor for Left (Follow Master)
  private final SparkMax m_backRight; // Main / Master Motor for Right
  private final SparkMax m_frontRight; // Slave Motor for Right (Follow Master)
  // Simulated Motors
  private final DCMotor m_leftGearbox;
  private final DCMotor m_rightGearbox;
  private final SparkMaxSim m_leftSim;
  private final SparkMaxSim m_rightSim;

  // Simulated Drive Train
  DifferentialDrivetrainSim m_driveTrainSim;

  // Simulated Encoders
  SparkRelativeEncoderSim m_leftEncoderSim;
  SparkRelativeEncoderSim m_rightEncoderSim;

  // Motor Configs
  private final SparkMaxConfig m_backLeftConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_frontLeftConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_backRightConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_frontRightConfig = new SparkMaxConfig();

  // Main drive function
  private final DifferentialDrive m_ddrive;

  // Encoders
  private final RelativeEncoder m_encoderBackLeft;
  private final RelativeEncoder m_encoderFrontLeft;
  private final RelativeEncoder m_encoderBackRight;
  private final RelativeEncoder m_encoderFrontRight;

  // Motor PID Controllers
  private final SparkClosedLoopController m_backLeftPIDController;
  private final SparkClosedLoopController m_backRightPIDController;

  // Current Idle mode
  private boolean isBrakeMode;

  // motor feedforward
  SimpleMotorFeedforward m_driveFeedForward =
      new SimpleMotorFeedforward(
          DriveConstants.ksDriveVolts,
          DriveConstants.kvDriveVoltSecondsPerMeter,
          DriveConstants.kaDriveVoltSecondsSquaredPerMeter);

  // ex:
  // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Read%20Encoder%20Values/src/main/java/frc/robot/Robot.java

  // Odometry class for tracking robot pose (position on field)
  private final DifferentialDrivePoseEstimator m_driveOdometry;

  // track robot field location for dashboard
  private Field2d field = new Field2d();

  // setup SysID for auto profiling
  private final SysIdRoutine m_sysIdRoutine;

  private boolean gyroZeroPending = true;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Init gyro
    m_Gyro = new AHRS(NavXComType.kMXP_SPI);
    // init motors
    // rio means built into the roboRIO
    m_backLeft = new SparkMax(CANConstants.MOTOR_BACK_LEFT_ID, SparkMax.MotorType.kBrushless);
    m_frontLeft = new SparkMax(CANConstants.MOTOR_FRONT_LEFT_ID, SparkMax.MotorType.kBrushless);
    m_frontRight = new SparkMax(CANConstants.MOTOR_FRONT_RIGHT_ID, SparkMax.MotorType.kBrushless);
    m_backRight = new SparkMax(CANConstants.MOTOR_BACK_RIGHT_ID, SparkMax.MotorType.kBrushless);
    // Create simulated motors
    m_leftGearbox = DCMotor.getNEO(2);
    m_rightGearbox = DCMotor.getNEO(2);
    m_leftSim = new SparkMaxSim(m_backLeft, m_leftGearbox);
    m_rightSim = new SparkMaxSim(m_backRight, m_rightGearbox);

    // setup simulation for gyro
    int gyroID = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
    SimGyroAngleHandler = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroID, "Yaw"));

    // Create the simulation model of our drivetrain.
    m_driveTrainSim =
        new DifferentialDrivetrainSim(
            // Create a linear system from our identification gains.
            // TODO: Update after profiling
            LinearSystemId.identifyDrivetrainSystem(
                DriveConstants.kvDriveVoltSecondsPerMeter,
                DriveConstants.kaDriveVoltSecondsSquaredPerMeter,
                DriveConstants.kvDriveVoltSecondsPerMeterAngular,
                DriveConstants.kaDriveVoltSecondsSquaredPerMeterAngular),
            DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
            DriveConstants.GEAR_RATIO, // x to 1 gearing reduction
            DriveConstants.kTrackwidthMeters, // Track Width
            DriveConstants.WHEEL_RADIUS, // Wheel Radius
            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    // setup simulated encoders
    m_leftEncoderSim = new SparkRelativeEncoderSim(m_backLeft);
    m_rightEncoderSim = new SparkRelativeEncoderSim(m_backRight);

    // invert motors
    m_backRightConfig.inverted(true);
    m_frontRightConfig.inverted(true);

    // setup main and secondary motors
    m_frontLeftConfig.follow(m_backLeft); // set front left to follow back left
    m_frontRightConfig.follow(m_backRight); // set front right to follow back right

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
    m_backLeftPIDController = m_backLeft.getClosedLoopController();
    m_backRightPIDController = m_backRight.getClosedLoopController();

    // configure encoders
    // RPM TO m/s
    m_backLeftConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    m_backRightConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    m_frontLeftConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    m_frontRightConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
    // rotations to meters
    m_backLeftConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    m_backRightConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    m_frontLeftConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    m_frontRightConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
    resetEncoders();

    // setup PID controllers
    configureMotorPIDControllers();

    // configure Odemetry
    m_driveOdometry =
        new DifferentialDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getRotation2d(),
            getPositionLeft(),
            getPositionRight(),
            new Pose2d());

    final PPLTVController m_driveController = new PPLTVController(0.02);
    // Setup Base AutoBuilder (Autonomous)
    AutoBuilder.configure(
        this::getPose, // Pose2d supplier
        this::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        this::getSpeeds, // A method for getting the chassis' current speed and direction
        this::setSpeeds, // A consumer that takes the desired chassis speed and direction
        m_driveController, // PPLTVController is the built in path following controller for
        // differential drive trains
        DriveConstants.autoConfig, // AutoConfig
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
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltage(voltage, voltage),
                null, // No log consumer, since data is recorded by URCL
                this));

    // burn config to motors
    m_backLeft.configure(
        m_backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_frontLeft.configure(
        m_frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_backRight.configure(
        m_backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_frontRight.configure(
        m_frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  private void configureMotorPIDControllers() {
    // setup velocity PID controllers (used by auto)
    // PID
    m_backLeftConfig.closedLoop.pid(
        DriveConstants.kPDriveVel,
        DriveConstants.kIDriveVel,
        DriveConstants.kDDriveVel,
        DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightConfig.closedLoop.pid(
        DriveConstants.kPDriveVel,
        DriveConstants.kIDriveVel,
        DriveConstants.kDDriveVel,
        DriveConstants.kDrivetrainVelocityPIDSlot);
    // Set Izone (Integral Zone)
    m_backLeftConfig.closedLoop.iZone(
        DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightConfig.closedLoop.iZone(
        DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
    // set output range
    m_backLeftConfig.closedLoop.outputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainVelocityPIDSlot);
    m_backRightConfig.closedLoop.outputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainVelocityPIDSlot);

    // setup position PID controllers (used when we manually path find)
    // PID
    m_backLeftConfig.closedLoop.pid(
        DriveConstants.kPDrivePos,
        DriveConstants.kIDrivePos,
        DriveConstants.kDDrivePos,
        DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightConfig.closedLoop.pid(
        DriveConstants.kPDrivePos,
        DriveConstants.kIDrivePos,
        DriveConstants.kDDrivePos,
        DriveConstants.kDrivetrainPositionPIDSlot);
    // Integral Zone

    m_backLeftConfig.closedLoop.iZone(
        DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightConfig.closedLoop.iZone(
        DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
    // Output Range
    m_backLeftConfig.closedLoop.outputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightConfig.closedLoop.outputRange(
        DriveConstants.kMinOutputDrive,
        DriveConstants.kMaxOutputDrive,
        DriveConstants.kDrivetrainPositionPIDSlot);
  }

  public void setVoltage(Voltage rightVoltage, Voltage leftVoltage) {
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

  /**
   * When this function is activated, the robot is in "drive straight" mode. Whatever direction the
   * robot was heading when "drive straight" mode was entered will be maintained. We use the current
   * pose of the robot to determine the direction. The robot will continue to move in this direction
   * until the function is canceled.
   */
  public Command driveStraight() {

    // get current pose
    Pose2d currentPose = getPose();
    // get current angle
    double currentAngle = currentPose.getRotation().getDegrees();
    // calculate wanted pose, add 2 meter to x value of current pose
    List<Pose2d> wantedPoses = new ArrayList<Pose2d>();
    wantedPoses.add(currentPose);
    wantedPoses.add(
        new Pose2d(
            currentPose.getTranslation().getX() + 5,
            currentPose.getTranslation().getY(),
            new Rotation2d(currentAngle)));
    wantedPoses.add(
        new Pose2d(
            currentPose.getTranslation().getX() + 10,
            currentPose.getTranslation().getY(),
            new Rotation2d(currentAngle)));
    // generate path
    return GenerateOnTheFlyCommand(wantedPoses);
  }

  /**
   * Builds a command to follow a path based on a given list of desired poses
   *
   * @param desiredPoses A list of poses for the robot to move to
   * @return The command to follow created path
   */
  public Command GenerateOnTheFlyCommand(List<Pose2d> desiredPoses) {
    // Creates the path to follow
    PathPlannerPath path = generateOnTheFlyPath(desiredPoses);
    // Returns built command following the path
    return AutoBuilder.followPath(path);
  }

  /**
   * Creates a path based on a given list of desired poses
   *
   * @param desiredPoses A list of poses for the robot to move to. Requires atleast 2 poses.
   * @return The path with the poses to go to.
   */
  private PathPlannerPath generateOnTheFlyPath(List<Pose2d> desiredPoses) {
    // Turn poses into waypoints
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(desiredPoses);
    // Create path with waypoints
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            DriveConstants.OnTheFly.kPathConstraints,
            new IdealStartingState(0, desiredPoses.get(0).getRotation()),
            new GoalEndState(0, desiredPoses.get(desiredPoses.size() - 1).getRotation()));
    // Disables the path being mirrored based on which alliance we are on
    path.preventFlipping = true;
    return path;
  }

  /**
   * This function can return our robots DiffernentialDriveWheelSpeeds, which is the speed of each
   * side of the robot.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
  }

  /**
   * This function can return our robots ChassisSpeeds, which is vx (m/s), vy (m/s), and omega
   * (rad/s).
   */
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * This function can set our robots ChassisSpeeds, which is vx (m/s), vy (m/s), and omega (rad/s).
   * vy is always 0 as we are not strafing.
   */
  public void setSpeeds(ChassisSpeeds speeds) {
    setWheelVelocities(DriveConstants.kDriveKinematics.toWheelSpeeds(speeds));
  }

  /**
   * This function can set our robots DifferentialDriveWheelSpeeds, which is the speed of each side
   * of the robot.
   */
  public void setWheelVelocities(DifferentialDriveWheelSpeeds speeds) {
    // get left and right speeds in m/s, and run through feedforward to get feedforward voltage
    // offset
    double leftSpeed = speeds.leftMetersPerSecond;
    double rightSpeed = speeds.rightMetersPerSecond;
    // set to position of motors
    m_backLeftPIDController.setReference(
        leftSpeed,
        SparkBase.ControlType.kVelocity,
        DriveConstants.kDrivetrainVelocityPIDSlot,
        m_driveFeedForward.calculate(leftSpeed));
    m_backRightPIDController.setReference(
        rightSpeed,
        SparkBase.ControlType.kVelocity,
        DriveConstants.kDrivetrainVelocityPIDSlot,
        m_driveFeedForward.calculate(rightSpeed));
  }

  // in meters, use averageDistance() to get average distance traveled, as an offset to set this
  // function.
  public void driveToPosition(final double NewPosition) {
    m_backLeftPIDController.setReference(
        NewPosition, SparkBase.ControlType.kPosition, DriveConstants.kDrivetrainPositionPIDSlot);
    m_backRightPIDController.setReference(
        NewPosition, SparkBase.ControlType.kPosition, DriveConstants.kDrivetrainPositionPIDSlot);
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

  public double currentDistance() {
    return getPose().getTranslation().getX();
  }

  public void SetBrakemode() {
    m_backLeftConfig.idleMode(IdleMode.kBrake);
    m_backRightConfig.idleMode(IdleMode.kBrake);
    m_frontLeftConfig.idleMode(IdleMode.kBrake);
    m_frontRightConfig.idleMode(IdleMode.kBrake);
    // reburn configs during runtime
    m_backLeft.configure(
        m_backLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_frontLeft.configure(
        m_frontLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_backRight.configure(
        m_backRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_frontRight.configure(
        m_frontRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    isBrakeMode = true;
  }

  public void SetCoastmode() {
    m_backLeftConfig.idleMode(IdleMode.kCoast);
    m_backRightConfig.idleMode(IdleMode.kCoast);
    m_frontLeftConfig.idleMode(IdleMode.kCoast);
    m_frontRightConfig.idleMode(IdleMode.kCoast);
    // reburn configs during runtime
    m_backLeft.configure(
        m_backLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_frontLeft.configure(
        m_frontLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_backRight.configure(
        m_backRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_frontRight.configure(
        m_frontRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    isBrakeMode = false;
  }

  public void SwitchBrakemode() {
    if (this.isBrakeMode) {
      this.SetCoastmode();
    } else {
      this.SetBrakemode();
    }
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
    return m_encoderBackLeft.getVelocity();
  }

  public double getVelocityRight() {
    return m_encoderBackRight.getVelocity();
  }

  public double getPositionLeft() {
    return m_encoderBackLeft.getPosition();
  }

  public double getPositionRight() {
    return m_encoderBackRight.getPosition();
  }

  @Override
  public void periodic() {
    if (gyroZeroPending && !m_Gyro.isCalibrating()) {
      resetGyro();
      gyroZeroPending = false;
    }
    // This method will be called once per scheduler run
    DifferentialDriveWheelSpeeds wheelSpeeds = this.getWheelSpeeds();
    SmartDashboard.putNumber("Left Encoder Speed (M/s)", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Encoder Speed (M/s)", wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Distance L", this.getPositionLeft());
    SmartDashboard.putNumber("Distance R", this.getPositionRight());
    SmartDashboard.putNumber("Average Distance Traveled", currentDistance());
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
    // link motors to simulation
    m_driveTrainSim.setInputs(
        m_leftSim.getAppliedOutput() * RobotController.getInputVoltage(),
        -m_rightSim.getAppliedOutput() * RobotController.getInputVoltage());
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveTrainSim.update(0.02);
    // update spark maxes
    m_leftSim.iterate(
        m_driveTrainSim.getLeftVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
    m_rightSim.iterate(
        m_driveTrainSim.getRightVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);
    // add load to battery
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveTrainSim.getCurrentDrawAmps()));
    // update sensors
    SimGyroAngleHandler.set(-m_driveTrainSim.getHeading().getDegrees());
    m_leftEncoderSim.setPosition(m_driveTrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setVelocity(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setPosition(m_driveTrainSim.getRightPositionMeters());
    m_rightEncoderSim.setVelocity(m_driveTrainSim.getRightVelocityMetersPerSecond());
  }
}
