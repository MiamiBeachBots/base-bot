// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.StraightCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;

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

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
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

  // misc init
  private JoystickButton m_switchCameraButton;
  private JoystickButton m_aimButton;
  private JoystickButton m_balanceButton;
  private JoystickButton m_straightButton;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // set default drive command
    m_driveSubsystem.setDefaultCommand(m_defaultDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // buttons
    m_switchCameraButton = new JoystickButton(m_controller1, Constants.SWAPCAMBUTTON);
    m_aimButton = new JoystickButton(m_flightStick, Constants.AIMBUTTON);
    m_balanceButton = new JoystickButton(m_controller1, Constants.BALANCEBUTTON);
    m_straightButton = new JoystickButton(m_controller1, Constants.STRAIGHTBUTTON);
    // commands
    m_balanceButton.whileTrue(m_balanceCommand);
    m_aimButton.whileTrue(m_aimCommand);
    m_straightButton.whileTrue(m_straightCommand);
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
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
