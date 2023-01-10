// Copyright (c) Jack Nelson & Miami Beach Bots

package frc.robot.subsystems;

// import motor & frc dependencies
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** This Subsystem is what allows the code to interact with the drivetrain of the robot. */
public class DriveSubsystem extends SubsystemBase {

  // math constants
  private final double DELTA = 0.05;

  private final double KP = 0.015;

  // motors
  private final WPI_VictorSPX m_backLeft;
  private final WPI_VictorSPX m_frontLeft;
  private final WPI_VictorSPX m_backRight;
  private final WPI_VictorSPX m_frontRight;

  // motors controllers
  private final MotorControllerGroup m_left;
  private final MotorControllerGroup m_right;

  // drive function
  private final DifferentialDrive m_ddrive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // init motors
    // rio means built into the roboRIO
    m_backLeft = new WPI_VictorSPX(Constants.MOTORBACKLEFTID);
    m_frontLeft = new WPI_VictorSPX(Constants.MOTORFRONTLEFTID);
    m_frontRight = new WPI_VictorSPX(Constants.MOTORFRONTRIGHTID);
    m_backRight = new WPI_VictorSPX(Constants.MOTORBACKRIGHTID);

    // init motors controllers
    m_left = new MotorControllerGroup(m_backLeft, m_frontLeft);
    m_right = new MotorControllerGroup(m_frontRight, m_backRight);

    // init drive function
    m_ddrive = new DifferentialDrive(m_left, m_right);
  }

  // default tank drive function
  // **tank drive = specific control style where two parallel forces of motion are controlled to
  // create linear and rotational motion
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_ddrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void backward() {
    this.tankDrive(0.5, -0.5);
  }

  public void stop() {
    this.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
