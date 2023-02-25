// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.StraightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
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
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  // The robots commands are defined here..
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final AutoCommand m_autoCommand = new AutoCommand(m_driveSubsystem, m_gyroSubsystem);
  private final AimCommand m_aimCommand = new AimCommand(m_driveSubsystem, m_gyroSubsystem);
  private final BalanceCommand m_balanceCommand =
      new BalanceCommand(m_driveSubsystem, m_gyroSubsystem);
  private final DefaultDrive m_defaultDrive =
      new DefaultDrive(m_driveSubsystem, m_controller1::getThrottle, m_controller1::getY);
  private final StraightCommand m_straightCommand =
      new StraightCommand(
          m_driveSubsystem, m_gyroSubsystem, m_controller1::getThrottle, m_controller1::getY);
  private final ClawCommand m_clawCommand = new ClawCommand(m_clawSubsystem);
  private final ArmUpCommand m_armUpCommand = new ArmUpCommand(m_armSubsystem);
  private final ArmDownCommand m_armDownCommand = new ArmDownCommand(m_armSubsystem);
  private final ArmExtendCommand m_armExtendCommand =
      new ArmExtendCommand(m_armSubsystem, m_flightStick::getY);
  // misc init
  private JoystickButton m_switchCameraButton;
  private JoystickButton m_aimButton;
  private JoystickButton m_balanceButton;
  private JoystickButton m_straightButton;
  private JoystickButton m_clawButton;
  private JoystickButton m_armUpButton;
  private JoystickButton m_armDownButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // set default drive command
    m_driveSubsystem.setDefaultCommand(m_defaultDrive);
    // set claw default command
    m_clawSubsystem.setDefaultCommand(m_clawCommand);
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
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            DriveConstants.driveTrajectoryConfig);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory, // The trajectory to follow
            m_driveSubsystem
                ::getPose, // A method for getting the current pose of the drive subsystem
            new RamseteController(
                DriveConstants.kRamseteB,
                DriveConstants
                    .kRamseteZeta), // A RamseteController that controls the drive subsystem's
            // motion
            DriveConstants
                .FeedForward, // A feedforward value to apply to the drive subsystem's control
            // signals
            DriveConstants.kDriveKinematics, // The kinematics of the drive subsystem
            m_driveSubsystem
                ::getWheelSpeeds, // A method for getting the current wheel speeds of the drive
            // subsystem
            new PIDController(
                DriveConstants.kPDriveVel,
                0,
                0), // A PID controller for left wheel velocity control
            new PIDController(
                DriveConstants.kPDriveVel,
                0,
                0), // A PID controller for right wheel velocity control
            // RamseteCommand passes volts to the callback
            m_driveSubsystem
                ::tankDriveVolts, // A tank drive method that accepts voltage values to control the
            // drive subsystem
            m_driveSubsystem // The drive subsystem to control
            );

    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));
  }
}
