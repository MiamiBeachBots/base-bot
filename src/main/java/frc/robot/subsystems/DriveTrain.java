// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import motors
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

// dependency import
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    // math constants
    private final double DELTA = 0.05;
    private final double KP = 0.015;

    // motors
    private Victor m_frontRight;
    private Victor m_backLeft;
    private Victor m_backRight;
    private Victor m_frontLeft;

    // motors controllers
    private MotorControllerGroup m_left;
    private MotorControllerGroup m_right;

    // drive function
    private DifferentialDrive m_ddrive;

    public DriveTrain() {
        // init motors
        m_frontRight = new Victor(Constants.MOTORFRONTRIGHTCHANNEL);
        m_backLeft = new Victor(Constants.MOTORBACKLEFTCHANNEL);
        m_backRight = new Victor(Constants.MOTORBACKRIGHTCHANNEL);
        m_frontLeft = new Victor(Constants.MOTORFRONTLEFTCHANNEL);

        // init motors controllers
        m_left = new MotorControllerGroup(m_backLeft, m_frontLeft);
        m_right = new MotorControllerGroup(m_frontRight, m_backRight);

        // init drive function
        m_ddrive = new DifferentialDrive(m_left, m_right);
    }

    // default tank drive function
    // **tank drive = specific control style where two parallel forces of motion are controlled to create linear and rotational motion
    public void defaultDrive(double leftSpeed, double rightSpeed) {
        m_ddrive.tankDrive(leftSpeed, rightSpeed);
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
