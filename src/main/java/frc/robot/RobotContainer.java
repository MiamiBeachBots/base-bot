// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.StraightCommand;
import frc.robot.commands.arm.ArmDownCommand;
import frc.robot.commands.arm.ArmExtendCommand;
import frc.robot.commands.arm.ArmUpCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.subsystems.arm.ClawSubsystem;
import frc.robot.subsystems.arm.ElevatorSubsystem;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import java.util.HashMap;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Init joysticks
  private final Joystick m_controller1 = new Joystick(Constants.CONTROLLERUSBINDEX);
  private final Joystick m_flightStick = new Joystick(Constants.FLIGHTSTICKUSBINDEX);

  // The robot's subsystems are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Init Gyro & ultrasonic
  private final UltrasonicSubsystem m_ultrasonic1 =
      new UltrasonicSubsystem(Constants.ULTRASONIC1PORT);

  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_gyroSubsystem);
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final ExtensionSubsystem m_extensionSubsystem = new ExtensionSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  // The robots commands are defined here..
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final AimCommand m_aimCommand = new AimCommand(m_driveSubsystem, m_gyroSubsystem);
  private final BalanceCommand m_balanceCommand =
      new BalanceCommand(m_driveSubsystem, m_gyroSubsystem);
  private final DefaultDrive m_defaultDrive =
      new DefaultDrive(m_driveSubsystem, m_controller1::getThrottle, m_controller1::getY);
  private final StraightCommand m_straightCommand =
      new StraightCommand(
          m_driveSubsystem, m_gyroSubsystem, m_controller1::getThrottle, m_controller1::getY);
  private final ClawCommand m_clawCommand = new ClawCommand(m_clawSubsystem);
  private final ArmUpCommand m_armUpCommand = new ArmUpCommand(m_elevatorSubsystem);
  private final ArmDownCommand m_armDownCommand = new ArmDownCommand(m_elevatorSubsystem);
  private final ArmExtendCommand m_armExtendCommand =
      new ArmExtendCommand(m_extensionSubsystem, m_flightStick::getY);
  // misc init
  private JoystickButton m_switchCameraButton;
  private JoystickButton m_aimButton;
  private JoystickButton m_balanceButton;
  private JoystickButton m_straightButton;
  private JoystickButton m_clawButton;
  private JoystickButton m_armUpButton;
  private JoystickButton m_armDownButton;
  // Init For Autonomous
  private RamseteAutoBuilder autoBuilder;
  private final HashMap<String, Command> autonomousEventMap = new HashMap<>();
  private SendableChooser<String> autoDashboardChooser = new SendableChooser<>();
  private List<PathPlannerTrajectory> pathGroup;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Initialize the autonomous command
    initializeAutonomous();

    // set default drive command
    m_driveSubsystem.setDefaultCommand(m_defaultDrive);
    // set claw default command
    m_clawSubsystem.setDefaultCommand(m_clawCommand);
    // set arm default command
    m_extensionSubsystem.setDefaultCommand(m_armExtendCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Controller buttons
    m_switchCameraButton = new JoystickButton(m_controller1, Constants.SWAPCAMBUTTON);
    m_balanceButton = new JoystickButton(m_controller1, Constants.BALANCEBUTTON);
    m_straightButton = new JoystickButton(m_controller1, Constants.STRAIGHTBUTTON);
    // Joystick buttons
    m_clawButton = new JoystickButton(m_flightStick, Constants.CLAWBUTTON);
    m_aimButton = new JoystickButton(m_flightStick, Constants.AIMBUTTON);
    m_armDownButton = new JoystickButton(m_flightStick, Constants.ARMDOWNBUTTON);
    m_armUpButton = new JoystickButton(m_flightStick, Constants.ARMUPBUTTON);
    // commands
    m_balanceButton.whileTrue(m_balanceCommand);
    m_aimButton.whileTrue(m_aimCommand);
    m_straightButton.whileTrue(m_straightCommand);
    m_clawButton.toggleOnTrue(m_clawCommand).toggleOnFalse(m_clawCommand);
    m_armDownButton.whileTrue(m_armDownCommand);
    m_armUpButton.whileTrue(m_armUpCommand);
  }

  private void initializeAutonomous() {
    // Network Table Routine Options
    autoDashboardChooser.setDefaultOption("Auto With Balancing", "FullAuto");
    autoDashboardChooser.addOption("Auto Without Balancing", "AutoNoBalance");
    autoDashboardChooser.addOption("Auto Test Mode", "Test");

    // Events
    // ex:
    // autonomousEventMap.put("A", new PathFollowingCommand(m_driveSubsystem, pathGroup.get(0)));
    autonomousEventMap.put("BalanceRobot", m_balanceCommand);

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every
    // time you want to create an auto command.
    // Use blue side of field when designing!
    autoBuilder =
        new RamseteAutoBuilder(
            m_driveSubsystem::getPose, // Pose2d supplier
            m_driveSubsystem
                ::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
            new RamseteController(
                DriveConstants.kRamseteB, DriveConstants.kRamseteZeta), // RamseteController
            DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
            DriveConstants
                .FeedForward, // A feedforward value to apply to the drive subsystem's controllers
            m_driveSubsystem
                ::getWheelSpeeds, // A method for getting the current wheel speeds of the drive
            new PIDConstants(
                DriveConstants.kPDriveVel, 0, 0), // A PID controller for wheel velocity control
            m_driveSubsystem
                ::tankDriveVolts, // A consumer that takes left and right wheel voltages and sets
            // them to the drive subsystem's controllers
            autonomousEventMap,
            true, // change for either team
            m_driveSubsystem //  Requirements of the commands (should be the drive subsystem)
            );
  }

  // for autonomous
  public DefaultDrive getDefaultDrive() {
    return m_defaultDrive;
  }
  // for autonomous
  public UltrasonicSubsystem getUltrasonic1() {
    return m_ultrasonic1;
  }
  // to swap camera type.
  public JoystickButton getCameraButton() {
    return m_switchCameraButton;
  }
  // for future SmartDashboard uses.
  public Joystick getController1() {
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
    pathGroup =
        PathPlanner.loadPathGroup(
            autoDashboardChooser.getSelected(), DriveConstants.autoPathConstraints);
    // Generate the auto command from the auto builder using the routine selected in the dashboard.
    return autoBuilder.fullAuto(pathGroup);
  }
}
