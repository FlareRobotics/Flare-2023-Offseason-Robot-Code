package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.math.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(Constants.enableSmartDashboard)
    {
      SmartDashboard.putBoolean("Vision Has Targets", LimelightHelpers.getTV("Limelight"));
      SmartDashboard.putNumber("AprilTag ID", LimelightHelpers.getFiducialID("Limelight"));
      SmartDashboard.putNumber("AprilTag Target X", LimelightHelpers.getTargetPose3d_RobotSpace("Limelight").getX());
      SmartDashboard.putNumber("AprilTag Target Y", LimelightHelpers.getTargetPose3d_RobotSpace("Limelight").getY());
      SmartDashboard.putNumber("AprilTag Target Z", LimelightHelpers.getTargetPose3d_RobotSpace("Limelight").getZ());
      SmartDashboard.putNumber("Vision Latency", LimelightHelpers.getLatency_Pipeline("Limelight") + LimelightHelpers.getLatency_Capture("Limelight"));
    }
  }

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
