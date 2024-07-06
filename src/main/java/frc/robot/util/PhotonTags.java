package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonTags {

  double CAMERA_HEIGHT_METERS = 0.20;
  double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

  static PhotonCamera camera = new PhotonCamera("AprilTags");
  static PhotonPipelineResult result = getLatestPipeline();
  static PhotonTrackedTarget t = getBestTarget(result);
  static int tagId;

  public static PhotonPipelineResult getLatestPipeline() {
    return camera != null? camera.getLatestResult() : null;
  }

  public double getPipelineToPose(){
    return getBestTarget(getLatestPipeline()).getYaw();
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  private static boolean hasTarget() {
    return result != null && result.hasTargets();
  }

  public boolean hasTag(){
    return hasTarget();
  }

  public static List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result != null ? result.getTargets() : List.of();
  }

  public static PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
    return result != null ? result.getBestTarget() : null;
  }

  public PhotonTrackedTarget getBestTarget() {
    return result != null ? result.getBestTarget() : null;
  }

  private static double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  private static double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public static double getYaw(){
    return getYaw();
  }

  public static double getPitch(){
    return getPitch();
  }

  public static List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  public static int getTargetId(PhotonTrackedTarget target) {
    return target != null? target.getFiducialId() : null;
  }

  public static double getPoseAbmiguity(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity();
  }

  public int getTargetCurrentId() {
    return getTargetId(t);
  }
  
  public static Transform3d getBestCamera(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }
  
  public Transform3d getCamera3d(){
    return getBestCamera(t);
  }
  
  public static void printToDashboard() {
    PhotonPipelineResult latestPipeline = getLatestPipeline();
    if (hasTarget()) {
      PhotonTrackedTarget bestTarget = getBestTarget(latestPipeline);
      if (t != null) {
        SmartDashboard.putNumber("TAG YAW", getYaw(bestTarget));
        SmartDashboard.putNumber("TAG PITCH", getPitch(bestTarget));
        SmartDashboard.putNumber("TAG ID", getTargetId(bestTarget));
      }
    }
  }
}
