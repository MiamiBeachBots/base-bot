// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
// import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
// TODO: Add gyro and ultrasonic.
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private UsbCamera camera1;
  private UsbCamera camera2;
  private VideoSink mainCameraServer;
  private int cameraCounter = 2;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    mainCameraServer = CameraServer.getServer();
    // Tell both cameras to always stream.
    // camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    // this is to put git info in the dashboard & Logs
    String deployDir = Filesystem.getDeployDirectory().getPath();
    String branchName = "unknown";
    String commitHash = "unknown";
    try {
      branchName = Files.readString(Path.of(deployDir, "branch.txt"));
      commitHash = Files.readString(Path.of(deployDir, "commit.txt"));

    } catch (IOException e) {
      e.printStackTrace();
      System.out.println("Parsing Git metadata Files Failed");
    }
    System.out.println("Branch: " + branchName);
    System.out.println("Commit: " + commitHash);
    SmartDashboard.putString("Branch", branchName);
    SmartDashboard.putString("Commit", commitHash);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // DO ultrasonid
    // SmartDashboard.putNumber("Auto Sensor", getRangeInches(ultrasonic1));
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer
        .getCameraButton()
        .whenPressed(
            new InstantCommand(
                () -> {
                  cameraCounter++;
                  if (cameraCounter % 2 == 0) {
                    System.out.println("Setting Camera 2");
                    mainCameraServer.setSource(camera2);
                  } else {
                    System.out.println("Setting Camera 1");
                    mainCameraServer.setSource(camera1);
                  }
                }));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("Ultrasonic info: " + m_robotContainer.getUltrasonic().get());
    // TODO: add dashboard stuff
    // SmartDashboard.putNumber("Ultrasonic Sensor Distance", getRangeInches(ultrasonic1));
    // SmartDashboard.putNumber("Ultrasonic Top Sensor Distance", getRangeInches(ultrasonic2));
    // SmartDashboard.putNumber("Throttle", m_robotContainer.getStick().getThrottle());
    // SmartDashboard.putNumber("Gyro Rate", m_robotContainer.getGyro().getRate());
    // SmartDashboard.putNumber("Gyro angle", m_robotContainer.getGyro().getAngle());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
