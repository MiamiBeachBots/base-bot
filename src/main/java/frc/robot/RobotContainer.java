// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem  m_driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final ShooterCommand m_shooterCommand =
      new ShooterCommand(m_shooterSubsystem, m_elevatorSubsystem, m_flightStick::getY);
  private final DefaultDrive m_defaultDrive = new DefaultDrive(m_driveSubsystem, m_controller1::getThrottle, m_controller1::getY);

  // misc init
  private JoystickButton m_switchCameraButton;

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
    // intake buttons
    JoystickButton m_intakeButtonBack = new JoystickButton(m_controller1, 8);
    JoystickButton m_intakeButtonFront = new JoystickButton(m_controller1, 7);
    JoystickButton m_intakeButton = new JoystickButton(m_controller1, 4);
    // shooter buttons
    JoystickButton m_shooterButton = new JoystickButton(m_flightStick, 1);
    // camera button
    m_switchCameraButton = new JoystickButton(m_controller1, 1);

    m_intakeButtonBack
        .whenPressed(new InstantCommand(() -> m_intakeSubsystem.backIntake(0.35)))
        .whenReleased(new InstantCommand(() -> m_intakeSubsystem.backIntake(0)));
    m_intakeButtonFront
        .whenPressed(new InstantCommand(() -> m_intakeSubsystem.frontIntake(0.45)))
        .whenReleased(new InstantCommand(() -> m_intakeSubsystem.frontIntake(0)));
    m_intakeButton
        .whenPressed(new InstantCommand(() -> m_intakeSubsystem.intake(-0.45)))
        .whenReleased(new InstantCommand(() -> m_intakeSubsystem.intake(0)));

    m_shooterButton.whenPressed(m_shooterCommand);
  }

  // for autonomous
  public DefaultDrive getDefaultDrive() {
    return m_defaultDrive;
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
