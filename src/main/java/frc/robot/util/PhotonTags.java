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

  public PhotonTags() {}

  public void setPitchCameraDegrees(double degrees) {
    this.CAMERA_PITCH_RADIANS = Units.degreesToRadians(degrees);
  }

  public static PhotonPipelineResult getLatestPipeline() {
    return camera.getLatestResult(); // Certifique-se de que este método não retorne null
  }

  public static PhotonCamera getCamera() {
    return camera;
  }

  public static boolean hasTarget(PhotonPipelineResult result) {
    return result != null && result.hasTargets();
  }

  public boolean hasTarget() {
    PhotonPipelineResult result = getLatestPipeline();
    return result != null && result.hasTargets();
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

  public static double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public static double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public static double getArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  public static double getSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  public static List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  public static int getTargetId(PhotonTrackedTarget target) {
    return target.getFiducialId();
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

  public static Transform3d getAlternateCamera(PhotonTrackedTarget target) {
    return target.getAlternateCameraToTarget();
  }

  public static void printToDashboard() {
    PhotonPipelineResult p = getLatestPipeline();
    if (hasTarget(p)) {
      PhotonTrackedTarget t = getBestTarget(p);
      if (t != null) {
        SmartDashboard.putNumber("TAG YAW", getYaw(t));
        SmartDashboard.putNumber("TAG AREA", getArea(t));
        SmartDashboard.putNumber("TAG PITCH", getPitch(t));
        SmartDashboard.putNumber("TAG ID", getTargetId(t));
      }
    }
  }
}
