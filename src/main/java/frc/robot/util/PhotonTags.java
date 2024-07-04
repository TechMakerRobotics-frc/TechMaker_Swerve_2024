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

  // Angle between horizontal and the camera.
  double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

  static PhotonPipelineResult p = getLatestPipeline();
  static PhotonTrackedTarget t = getBestTarget(p);
  static PhotonCamera camera = new PhotonCamera("AprilTags");
  static PhotonPipelineResult result = camera.getLatestResult();
  static int tagId;

  public PhotonTags() {}

  public void setPitchCameraDegrees(double degrees) {
    this.CAMERA_PITCH_RADIANS = Units.degreesToRadians(degrees);
  }
  // Have to use the same pipeline result each time you want to gather data.

  // Gets the processed data from the camera
  public static PhotonPipelineResult getLatestPipeline() {
    return result;
  }

  public static PhotonCamera getCamera() {
    return camera;
  }

  // Checks if there is a target in vision
  public static boolean hasTarget(PhotonPipelineResult result) {
    return result.hasTargets();
  }

  public boolean hasTarget() {
    PhotonPipelineResult result = getLatestPipeline();
    return result.hasTargets();
  }

  public static List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result.getTargets();
  }

  public static PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
    return result.getBestTarget();
  }

  public PhotonTrackedTarget getBestTarget() {
    return result.getBestTarget();
  }

  // The yaw of the target in degrees (positive right)
  public static double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  // The pitch of the target in degrees (positive up)
  public static double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  // The area (how much of the camera feed the bounding box takes up) as a percent
  // (0-100)
  public static double getArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  // The skew of the target in degrees (counter-clockwise positive)
  public static double getSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  // The 4 corners of the minimum bounding box rectangle
  public static List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  // The camera to target transform (Pose)
  // For some reason cannot get pose for reflectiveTape
  // public Transform2d getPose(PhotonTrackedTarget target){
  // return target.getCameraToTarget();
  // }

  // Get id of tag
  public static int getTargetId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  // How ambiguous the pose is????
  public static double getPoseAbmiguity(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity();
  }

  public int getTargetCurrentId() {
    return getTargetId(t);
  }

  /*
   * Get the transform that maps camera space (X = forward, Y = left, Z = up)
   * to object/fiducial tag space (X forward, Y left, Z up) with the lowest
   * reprojection error
   */
  public static Transform3d getBestCamera(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }

  /*
   * Get the transform that maps camera space (X = forward, Y = left, Z = up)
   * to object/fiducial tag space (X forward, Y left, Z up) with the lowest
   * highest error
   */
  public static Transform3d getAlternateCamera(PhotonTrackedTarget target) {
    return target.getAlternateCameraToTarget();
  }

  public static void printToDashboard() {
    PhotonPipelineResult p = getLatestPipeline();
    if (hasTarget(p)) {
      PhotonTrackedTarget t = getBestTarget(p);
      SmartDashboard.putNumber("TAG YAW", getYaw(t));
      SmartDashboard.putNumber("TAG AREA", getArea(t));
      SmartDashboard.putNumber("TAG PITCH", getPitch(t));
      SmartDashboard.putNumber("TAG ID", getTargetId(t));
    }
  }
}
