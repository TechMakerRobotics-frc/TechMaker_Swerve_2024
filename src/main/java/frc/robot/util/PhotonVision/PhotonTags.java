package frc.robot.util.PhotonVision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Height;
import frc.robot.util.UtilConstants.VisionConstants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonTags {

  double CAMERA_HEIGHT_METERS = 0.20;
  double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

  static int tagId;
  static PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_A_NAME);
  PhotonPipelineResult result = getLatestPipeline();
  PhotonTrackedTarget t = getBestTarget(result);

  /**
   * get the latest result of the camera.
   *
   * @return camera latest result
   */
  public static PhotonPipelineResult getLatestPipeline() {
    return camera.getLatestResult();
  }

  public static PhotonCamera getCamera() {
    return camera;
  }

  // Checks if there is a target in vision
  public static boolean hasTarget(PhotonPipelineResult result) {
    return result.hasTargets();
  }

  public static List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result.getTargets();
  }

  public static PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
    return result.getBestTarget();
  }

  /**
   * @param target the used target
   * @return The yaw of the target in degrees (positive right)
   */
  public static double getYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  /**
   * Method not static.
   *
   * @param target the used target
   * @return The yaw of the target in degrees (positive right)
   */
  public double getYawNStatic(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  /**
   * @param target the used target
   * @return The pitch of the target in degrees (positive up)
   */
  public static double getPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  /**
   * Method not static.
   *
   * @param target the used target
   * @return The pitch of the target in degrees (positive up)
   */
  public double getPitchNStatic(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  /**
   * @param target the used target
   * @return The area (how much of the camera feed the bounding box takes up) as a percent (0-100)
   */
  public static double getArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  /**
   * @param target the used target
   * @return The skew of the target in degrees (counter-clockwise positive)
   */
  public static double getSkew(PhotonTrackedTarget target) {
    return target.getSkew();
  }

  // The 4 corners of the minimum bounding box rectangle
  public static List<TargetCorner> getBoundingCorners(PhotonTrackedTarget target) {
    return target.getDetectedCorners();
  }

  // Get id of tag
  public static int getTargetId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  // How ambiguous the pose is????
  public static double getPoseAbmiguity(PhotonTrackedTarget target) {
    return target.getPoseAmbiguity();
  }

  /**
   * The transform that maps camera space (X = forward, Y = left, Z = up) to object/fiducial tag
   * space (X forward, Y left, Z up) with the lowest reprojection error.
   *
   * @param target the used target
   * @return Best camera to target
   */
  public static Transform3d getBestCamera(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget();
  }

  /**
   * Get the transform that maps camera space (X = forward, Y = left, Z = up) to object/fiducial tag
   * space (X forward, Y left, Z up) with the lowest highest error
   *
   * @param target the used target
   * @rerturn Alternate camera to target
   */
  public static Transform3d getAlternateCamera(PhotonTrackedTarget target) {
    return target.getAlternateCameraToTarget();
  }

  /**
   * Sets the used tag.
   *
   * @param tagNumber Number to compare
   * @return true if number is equal to current Tag
   */
  public static boolean hasUsedPipeline(int tagNumber) {
    PhotonPipelineResult p = getLatestPipeline();
    PhotonTrackedTarget t = getBestTarget(p);
    if (PhotonTags.hasTarget(p) && PhotonTags.getTargetId(t) == tagNumber) {
      return true;
    }
    return false;
  }

  public static double getDistanceToSpeaker() {
    if (!hasTarget(getLatestPipeline())) {
      return 0;
    } else {
      // a1 = LL panning angle
      // a2 = additional angle to target
      // tan(a1 + a2) = h/d
      // d = h/tan(a1+a2)
      double a2 = getPitch(getBestTarget(getLatestPipeline()));
      double a1 = VisionConstants.CAMERA_A_ANGLE;
      double h1 = VisionConstants.CAMERA_A_HEIGHT;
      double h2 = 1.4511; // Place holder Height of target

      double angleToGoal = (a1 + a2);
      double angleToGoalRadian = Math.toRadians(angleToGoal);

      return (h2 - h1) / Math.tan(angleToGoalRadian);
    }
  }

  public static double getDistanceToBase() {
    if (!hasTarget(getLatestPipeline())) {
      return 0;
    } else {
      // a1 = LL panning angle
      // a2 = additional angle to target
      // tan(a1 + a2) = h/d
      // d = h/tan(a1+a2)
      double a2 = getPitch(getBestTarget(getLatestPipeline()));
      double a1 = VisionConstants.CAMERA_A_ANGLE;
      double h1 = VisionConstants.CAMERA_A_HEIGHT;
      double h2 = 1.4511; // Place holder Height of target

      double angleToGoal = (a1 + a2);
      double angleToGoalRadian = Math.toRadians(angleToGoal);
      double targetHeight = h2 - h1;
      double distance = targetHeight / Math.tan(angleToGoalRadian);
      double cateto = Math.sqrt((Math.pow(distance, 2) - Math.pow(targetHeight, 2)));
      return cateto;
    }
  }

  public static double getDistance(String cameraName, int tag) {
    if (!hasTarget(getLatestPipeline())) {
      return 0;
    } else if (hasUsedPipeline(tag)) {
      // a1 = LL panning angle
      // a2 = additional angle to target
      // tan(a1 + a2) = h/d
      // d = h/tan(a1+a2)
      double a2 = getYaw(getBestTarget(getLatestPipeline()));
      double a1 = Height.getCameraAngle(cameraName);
      double h1 = Height.getCameraHeight(cameraName);
      double h2 = Height.getTagHeight(tag); // Place holder Height of target

      double angleToGoal = (a1 + a2);
      double angleToGoalRadian = Math.toRadians(angleToGoal);

      return (h2 - h1) / Math.tan(angleToGoalRadian);
    }
    return 0;
  }
}
