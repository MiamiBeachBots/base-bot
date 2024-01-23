// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.StraightCommand;
import frc.robot.commands.UltrasonicShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // These states are used to pass data between commands.
  private final ShooterState m_shooterState = new ShooterState();

  // Init joysticks
  private final CommandXboxController m_controller1 =
      new CommandXboxController(Constants.CONTROLLERUSBINDEX);
  private final Joystick m_flightStick = new Joystick(Constants.FLIGHTSTICKUSBINDEX);

  // The robot's subsystems are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Init Gyro & ultrasonic
  private final UltrasonicSubsystem m_ultrasonicShooterSubsystem =
      new UltrasonicSubsystem(Constants.ULTRASONICSHOOTERPORT);

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem(m_driveSubsystem);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ShooterSubsystem m_shooterSubsytem = new ShooterSubsystem();
  // The robots commands are defined here..
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final AimCommand m_aimCommand = new AimCommand(m_driveSubsystem, m_cameraSubsystem);
  private final BalanceCommand m_balanceCommand = new BalanceCommand(m_driveSubsystem);
  private final DefaultDrive m_defaultDrive =
      new DefaultDrive(m_driveSubsystem, this::getControllerLeftY, this::getControllerRightY);
  private final StraightCommand m_straightCommand =
      new StraightCommand(m_driveSubsystem, this::getControllerLeftY, this::getControllerRightY);
  private final UltrasonicShooterCommand m_ultrasonicShooterCommand =
      new UltrasonicShooterCommand(m_ultrasonicShooterSubsystem, m_shooterState);
  private final ArmCommand m_armCommand =
      new ArmCommand(m_armSubsystem, m_shooterState, this::GetFlightStickY);
  private final ShooterCommand m_shooterCommand = new ShooterCommand(m_shooterSubsytem);
  private Command m_driveToSpeaker;
  // Init Buttons
  private Trigger m_switchCameraButton;
  private Trigger m_balanceButton;
  private Trigger m_straightButton;
  private Trigger m_brakeButton;
  private Trigger m_coastButton;
  private JoystickButton m_aimButton;
  private JoystickButton m_fireButton;
  private Trigger m_driveToSpeakerButton;
  // Init For Autonomous
  // private RamseteAutoBuilder autoBuilder;
  private SendableChooser<String> autoDashboardChooser = new SendableChooser<String>();
  public final boolean enableAutoProfiling = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize the autonomous command
    initializeAutonomous();
    // Setup On the Fly Path Planning
    configureTeleopPaths();
    // Configure the button bindings
    setupTriggers();
    // Bind the commands to the triggers
    if (enableAutoProfiling) {
      bindDriveSysIDCommands();
      // bindArmSysIDCommands();
      // bindShooterSysIDCommands();
    } else {
      bindCommands();
    }

    // set default drive command
    m_driveSubsystem.setDefaultCommand(m_defaultDrive);
    // set default command for shooter ultrasonic sensor
    m_ultrasonicShooterSubsystem.setDefaultCommand(m_ultrasonicShooterCommand);
    // set default command for arm
    m_armSubsystem.setDefaultCommand(m_armCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void setupTriggers() {
    // Controller buttons
    m_switchCameraButton = m_controller1.x();
    m_brakeButton = m_controller1.a();
    m_coastButton = m_controller1.b();
    m_balanceButton = m_controller1.rightBumper();
    m_straightButton = m_controller1.rightTrigger();
    m_driveToSpeakerButton = m_controller1.y();
    // Joystick buttons
    m_aimButton = new JoystickButton(m_flightStick, Constants.AIMBUTTON);
    m_fireButton = new JoystickButton(m_flightStick, Constants.FIREBUTTON);
  }

  private void bindCommands() {
    // commands
    m_balanceButton.whileTrue(m_balanceCommand);
    m_straightButton.whileTrue(m_straightCommand);
    m_aimButton.whileTrue(m_aimCommand);
    m_fireButton.whileTrue(m_shooterCommand);
    m_driveToSpeakerButton.whileTrue(m_driveToSpeaker);

    m_brakeButton.whileTrue(new InstantCommand(() -> m_driveSubsystem.SetBrakemode()));
    m_coastButton.whileTrue(new InstantCommand(() -> m_driveSubsystem.SetCoastmode()));
  }

  private void bindDriveSysIDCommands() {
    m_controller1.a().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_controller1.b().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_controller1.x().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_controller1.y().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_controller1.leftTrigger().whileTrue(new InstantCommand(() -> DataLogManager.stop()));
  }

  private void bindArmSysIDCommands() {
    m_controller1.a().whileTrue(m_armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_controller1.b().whileTrue(m_armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_controller1.x().whileTrue(m_armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_controller1.y().whileTrue(m_armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_controller1.leftTrigger().whileTrue(new InstantCommand(() -> DataLogManager.stop()));
  }

  private void bindShooterSysIDCommands() {
    m_controller1
        .a()
        .whileTrue(m_shooterSubsytem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_controller1
        .b()
        .whileTrue(m_shooterSubsytem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_controller1.x().whileTrue(m_shooterSubsytem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_controller1.y().whileTrue(m_shooterSubsytem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_controller1.leftTrigger().whileTrue(new InstantCommand(() -> DataLogManager.stop()));
  }

  private void initializeAutonomous() {
    // Network Table Routine Options
    autoDashboardChooser.setDefaultOption("Auto With Balancing", "FullAuto");
    autoDashboardChooser.setDefaultOption("DriveForward", "DriveForward");
    autoDashboardChooser.addOption("End at cones", "EndAtCones");
    autoDashboardChooser.addOption("Do Nothing", "DoNothing");
    SmartDashboard.putData(autoDashboardChooser);

    // Named Commands
    // ex:
    // NamedCommands.registerCommand("A", new PathFollowingCommand(m_driveSubsystem,
    // pathGroup.get(0)));
    NamedCommands.registerCommand("BalanceRobot", m_balanceCommand);
    NamedCommands.registerCommand(
        "BrakeCommand", new InstantCommand(() -> m_driveSubsystem.SetBrakemode()));

    // autoBuilder =
    //     new RamseteAutoBuilder(
    //         m_driveSubsystem::getPose, // Pose2d supplier
    //         m_driveSubsystem
    //             ::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    //         new RamseteController(
    //             DriveConstants.kRamseteB, DriveConstants.kRamseteZeta), // RamseteController
    //         DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
    //         DriveConstants
    //             .FeedForward, // A feedforward value to apply to the drive subsystem's
    // controllers
    //         m_driveSubsystem::getWheelSpeeds, // A method for getting the current wheel speeds of
    // the drive
    //         new PIDConstants(
    //             DriveConstants.kPDriveVel, 0, 0), // A PID controller for wheel velocity control
    //         m_driveSubsystem
    //             ::tankDriveVolts, // A consumer that takes left and right wheel voltages and sets
    //         // them to the drive subsystem's controllers
    //         autonomousEventMap,
    //         true, // change for either team
    //         m_driveSubsystem //  Requirements of the commands (should be the drive subsystem)
    //         );
  }

  private void configureTeleopPaths() {
    // Limits for all Paths
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    PathPlannerPath speakerPath = PathPlannerPath.fromPathFile("TeleopSpeakerPath");

    m_driveToSpeaker = AutoBuilder.pathfindThenFollowPath(speakerPath, constraints);
  }

  public double getControllerRightY() {
    return -m_controller1.getRightY();
  }

  public double getControllerLeftY() {
    return -m_controller1.getLeftY();
  }

  public double GetFlightStickY() {
    return m_flightStick.getY();
  }

  // for autonomous
  public DefaultDrive getDefaultDrive() {
    return m_defaultDrive;
  }

  // to swap camera type.
  public Trigger getCameraButton() {
    return m_switchCameraButton;
  }

  // for future SmartDashboard uses.
  public CommandXboxController getController1() {
    return this.m_controller1;
  }

  // for smart dashboard.
  public Joystick getFlightStick() {
    return this.m_flightStick;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // get the name of the auto from network tables, as the rest is preconfigured by the drive
    // subsystem.
    String autoName = autoDashboardChooser.getSelected();
    return new PathPlannerAuto(autoName);
  }
}
