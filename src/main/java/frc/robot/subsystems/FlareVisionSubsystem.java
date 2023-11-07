package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareVisionSubsystem extends SubsystemBase {

  public static PhotonCamera camera;
  public static WPI_VictorSPX ledSpx = new WPI_VictorSPX(8);

  public FlareVisionSubsystem()
  {
    camera = new PhotonCamera("Camera");
    FlareVisionSubsystem.ledSpx.set(0);
  }

  

  @Override
  public void periodic() {
    camera = new PhotonCamera("Camera");
    if(camera.getLatestResult() != null)
    {
      SmartDashboard.putBoolean("Has Targets", camera.getLatestResult().hasTargets());
      if(camera.getLatestResult().hasTargets())
      {
        SmartDashboard.putNumber("X Camera", camera.getLatestResult().getBestTarget().getYaw());
        SmartDashboard.putNumber("Y Camera", camera.getLatestResult().getBestTarget().getPitch());
      }
    }
  }

  public static PhotonTrackedTarget getBestTarget() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result == null)
      return null;

    if (result.hasTargets()) {
      return result.getBestTarget();
    } else {
      return null;
    }
  }



  public static double getDistanceToGoal(PhotonTrackedTarget besTarget) {
    return besTarget == null ? 0 : besTarget.getYaw();
  }

  public static double getYdistance(PhotonTrackedTarget besTarget) {
    return besTarget == null ? 0 : besTarget.getPitch();
  }
}