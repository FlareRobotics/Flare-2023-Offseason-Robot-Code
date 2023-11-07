package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareVisionSubsystem extends SubsystemBase {

  public static PhotonCamera camera;
  public static VictorSP ledSpx = new VictorSP(5);

  public FlareVisionSubsystem()
  {
    camera = new PhotonCamera("Camera");
  }

  

  @Override
  public void periodic() {
    
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