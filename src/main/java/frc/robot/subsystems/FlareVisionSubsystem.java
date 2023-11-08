package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlareVisionSubsystem extends SubsystemBase {

  public static PhotonCamera camera;
  public static VictorSP ledSpx = new VictorSP(5);

  public FlareVisionSubsystem()
  {
    camera = new PhotonCamera("Camera");
  }

  

  @Override
  public void periodic() {
    if(!Constants.enableSmartDashboard)
      return;

    if(camera != null && camera.getLatestResult() != null)
    {
      SmartDashboard.putBoolean("Has Target", camera.getLatestResult().hasTargets());

      if(camera.getLatestResult().hasTargets())
      {
        SmartDashboard.putNumber("Target X", camera.getLatestResult().getBestTarget().getYaw());
        SmartDashboard.putNumber("Target Y", camera.getLatestResult().getBestTarget().getPitch());
      }
    }
  }

  public static PhotonTrackedTarget getBestTarget() {
    if(camera == null)
    return null;

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