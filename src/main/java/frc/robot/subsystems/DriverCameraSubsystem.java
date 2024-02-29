package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverCameraSubsystem extends SubsystemBase {
  private final UsbCamera m_DriverCamera1;
  private final int k_FrontCameraID = 0;
  private NetworkTableEntry cameraSelection;

  // Initalizes DriverCameraSubsystem
  public DriverCameraSubsystem() {
    // connect to network tables
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    // Connect to Cameras
    m_DriverCamera1 = CameraServer.startAutomaticCapture(k_FrontCameraID);

    // set the camera selection to the front camera
    cameraSelection.setString(m_DriverCamera1.getName());
    // start telemetry
    // m_DriverCamera1.set
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Driver Camera FPS", m_DriverCamera1.getActualFPS());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
