// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterState.ShooterModes;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FlywheelCommand;
// import frc.robot.commands.LifterCommand;
import frc.robot.commands.StraightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
// import frc.robot.subsystems.LifterSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
      new CommandXboxController(Constants.CONTROLLER_USB_INDEX);
  private final Joystick m_flightStick = new Joystick(Constants.FLIGHTSTICK_USB_INDEX);

  // The robot's subsystems are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Init Gyro & ultrasonic
  // private final UltrasonicSubsystem m_ultrasonicShooterSubsystem =
  //    new UltrasonicSubsystem(Constants.ULTRASONIC_SHOOTER_PORT); // TODO: Color sensor

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem(m_driveSubsystem);
  private final FlywheelSubsystem m_shooterSubsytem = new FlywheelSubsystem();
  // private final LifterSubsystem m_leftLifterSubsystem =
  // new LifterSubsystem(Constants.CANConstants.MOTOR_LIFTER_LEFT_ID);
  // private final LifterSubsystem m_rightLifterSubsystem =
  // new LifterSubsystem(Constants.CANConstants.MOTOR_LIFTER_RIGHT_ID);
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  // The robots commands are defined here..
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final AimCommand m_aimCommand =
      new AimCommand(m_driveSubsystem, m_cameraSubsystem, m_shooterState);
  private final DefaultDrive m_defaultDrive =
      new DefaultDrive(
          m_driveSubsystem, m_shooterState, this::getControllerLeftY, this::getControllerRightY);
  private final StraightCommand m_straightCommand = new StraightCommand(m_driveSubsystem);
  private final FlywheelCommand m_shooterCommand =
      new FlywheelCommand(m_shooterSubsytem, m_shooterState);
  // private final LifterCommand m_LeftLifterCommand = new LifterCommand(m_leftLifterSubsystem);
  // private final LifterCommand m_RightLifterCommand = new LifterCommand(m_rightLifterSubsystem);
  private final ElevatorCommand m_ElevatorCommand =
      new ElevatorCommand(m_ElevatorSubsystem, m_shooterState);
  private final ArmCommand m_ArmCommand = new ArmCommand(m_ArmSubsystem, m_shooterState);

  // EX: these commands are used by autonomous only
  // private final AimAmpCommand m_AimAmpCommand = new AimAmpCommand(m_armSubsystem,
  // m_shooterState);

  // this command is used by on the fly path planning
  // private Command m_driveToAmp;
  // Init Buttons
  // private Trigger m_balanceButton;
  private Trigger m_straightButton;
  private Trigger m_toggleBrakeButton;
  private Trigger m_aimButton;
  private Trigger m_switchQueuedButton;
  private Trigger m_defaultButton_driver;
  // private Trigger m_lifterRightButton;
  // private Trigger m_lifterLeftButton;
  // private Trigger m_driveToAmpButton;
  // private Trigger m_lifterDirectionButton;
  // joystick buttons
  private JoystickButton m_defaultButton;
  private JoystickButton m_troughButton;
  private JoystickButton m_bargeButton;
  private JoystickButton m_intakeButton;
  private JoystickButton m_shooterTrigger;
  private JoystickButton m_reefT2Button;
  private JoystickButton m_reefT3Button;
  private JoystickButton m_reefT4Button;
  private JoystickButton m_processorButton;
  // Init For Autonomous
  private LoggedDashboardChooser<String> autoDashboardChooser =
      new LoggedDashboardChooser<String>("AutoMode");

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
      // bindDriveSysIDCommands();
      bindArmSysIDCommands();
      // bindElevatorSysIDCommands();
    } else {
      bindCommands();
    }

    // set default drive command
    m_driveSubsystem.setDefaultCommand(m_defaultDrive);

    // set default commands for other components
    m_ElevatorSubsystem.setDefaultCommand(m_ElevatorCommand);
    m_ArmSubsystem.setDefaultCommand(m_ArmCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void setupTriggers() {
    // Controller buttons
    m_toggleBrakeButton = m_controller1.b();
    m_straightButton = m_controller1.leftBumper();
    m_aimButton = m_controller1.rightBumper();
    m_switchQueuedButton = m_controller1.y();
    m_defaultButton_driver = m_controller1.a();

    // Joystick buttons
    m_defaultButton = new JoystickButton(m_flightStick, Constants.DEFAULT_BUTTON);
    m_troughButton = new JoystickButton(m_flightStick, Constants.TROUGH_BUTTON);
    m_bargeButton = new JoystickButton(m_flightStick, Constants.BARGE_BUTTON);
    m_processorButton = new JoystickButton(m_flightStick, Constants.PROCESSOR_BUTTON);
    m_intakeButton = new JoystickButton(m_flightStick, Constants.INTAKE_BUTTON);
    m_reefT2Button = new JoystickButton(m_flightStick, Constants.REEFT2_BUTTON);
    m_reefT3Button = new JoystickButton(m_flightStick, Constants.REEFT3_BUTTON);
    m_reefT4Button = new JoystickButton(m_flightStick, Constants.REEFT4_BUTTON);
    // load and shoot buttons
    m_shooterTrigger = new JoystickButton(m_flightStick, Constants.TRIGGER);
  }

  private void bindCommands() {
    // commands
    m_straightButton.whileTrue(m_straightCommand);
    m_switchQueuedButton.whileTrue(new InstantCommand(() -> m_shooterState.switchModes()));
    m_defaultButton.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.DEFAULT)));
    m_defaultButton_driver.whileTrue(new InstantCommand(() -> m_shooterState.defaultOverride()));
    m_troughButton.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.TROUGH)));
    m_bargeButton.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.BARGE)));
    m_processorButton.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.PROCESSOR)));
    m_intakeButton.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.INTAKE)));
    m_reefT2Button.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.REEFT2)));
    m_reefT3Button.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.REEFT3)));
    m_reefT4Button.whileTrue(
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.REEFT4)));

    m_aimButton.whileTrue(m_aimCommand);

    m_toggleBrakeButton.whileTrue(new InstantCommand(() -> m_driveSubsystem.SwitchBrakemode()));
    // shooter + arm commands
    m_shooterTrigger.whileTrue(m_shooterCommand);
  }

  private void bindDriveSysIDCommands() {
    m_controller1.a().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_controller1.b().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_controller1.x().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_controller1.y().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_controller1.leftTrigger().whileTrue(new InstantCommand(() -> DataLogManager.stop()));
  }

  private void bindElevatorSysIDCommands() {
    m_controller1
        .a()
        .whileTrue(m_ElevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_controller1
        .b()
        .whileTrue(m_ElevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_controller1.x().whileTrue(m_ElevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_controller1.y().whileTrue(m_ElevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_controller1.leftTrigger().whileTrue(new InstantCommand(() -> DataLogManager.stop()));
    m_ElevatorSubsystem.disablePID();
  }

  private void bindArmSysIDCommands() {
    m_controller1.a().whileTrue(m_ArmSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_controller1.b().whileTrue(m_ArmSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_controller1.x().whileTrue(m_ArmSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_controller1.y().whileTrue(m_ArmSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_controller1.leftTrigger().whileTrue(new InstantCommand(() -> DataLogManager.stop()));
    m_ArmSubsystem.disablePID();
  }

  private void initializeAutonomous() {
    // Network Table Routine Options
    autoDashboardChooser.addDefaultOption("ReefCenter", "ReefCenter");
    autoDashboardChooser.addOption("ReefLeft", "ReefLeft");
    autoDashboardChooser.addOption("ReefRight", "ReefRight");
    autoDashboardChooser.addOption("DriveStraight", "DriveStraight");
    autoDashboardChooser.addOption("Do Nothing", "DoNothing");
    SmartDashboard.putData(autoDashboardChooser.getSendableChooser());

    // Named Commands
    // ex:
    // NamedCommands.registerCommand("A", new PathFollowingCommand(m_driveSubsystem,
    // pathGroup.get(0)));
    NamedCommands.registerCommand(
        "BrakeCommand", new InstantCommand(() -> m_driveSubsystem.SetBrakemode()));
    NamedCommands.registerCommand("ShooterCommand", m_shooterCommand);
    NamedCommands.registerCommand("AimCommand", m_aimCommand);
    NamedCommands.registerCommand(
        "IntakeCommand",
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.INTAKE)));
    NamedCommands.registerCommand(
        "BargeCommand", new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.BARGE)));
    NamedCommands.registerCommand(
        "TroughCommand",
        new InstantCommand(() -> m_shooterState.setQueuedMode(ShooterModes.TROUGH)));
    NamedCommands.registerCommand(
        "SwitchQueuedCommand", new InstantCommand(() -> m_shooterState.switchModes()));
  }

  private void configureTeleopPaths() {
    // TODO: Write new paths
    // EX
    // PathPlannerPath ampPath = PathPlannerPath.fromPathFile("TeleopAmpPath");

    // m_driveToAmp = AutoBuilder.pathfindThenFollowPath(ampPath, constraints);
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
    String autoName = autoDashboardChooser.get();
    return new PathPlannerAuto(autoName);
  }

  public void periodic() {
    // This method will be called once per scheduler run (Only for inter subsystem state updating)
    m_shooterState.StatePeriodic();
  }
}
