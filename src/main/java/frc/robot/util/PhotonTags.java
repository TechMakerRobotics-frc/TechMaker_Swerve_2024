package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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
  private Drive drive;
  static int tagId;
  private double targetHeight;
  private double goal;

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  final double P_GAIN = 0.1;
  final double D_GAIN = 0.0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  public PhotonTags(int TagId, double goal, Drive drivetrain) {
    drive = drivetrain;
    tagId = TagId;
    this.goal = goal;
    // aling();
    // go();
  }

  public void aling() {
    double rotationSpeed;
    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0;
    }
    boolean isFlipped =
        DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
    Rotation2d fieldRelativeRotation =
        isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, rotationSpeed * drive.getMaxAngularSpeedRadPerSec(), fieldRelativeRotation);

    drive.runVelocity(fieldRelativeSpeeds);
  }

  public void go() {
    double forwardSpeed;
    double targetHeightMeters = getTagHeight(tagId);
    if (result.hasTargets()) {
      // First calculate range
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              targetHeightMeters,
              CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

      forwardSpeed = range * drive.getMaxLinearSpeedMetersPerSec();
      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -controller.calculate(range, goal);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
  }

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

  public static boolean hasTarget() {
    PhotonPipelineResult result = getLatestPipeline();
    return result.hasTargets();
  }

  public static List<PhotonTrackedTarget> getTargets(PhotonPipelineResult result) {
    return result.getTargets();
  }

  public static PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
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

  private double getTagHeight(int tagId) {
    if (tagId == 1 || tagId == 2 || tagId == 5 || tagId == 6 || tagId == 9 || tagId == 10) {
      targetHeight = Units.inchesToMeters(53.38);
    } else if (tagId == 3 || tagId == 4 || tagId == 7 || tagId == 8) {
      targetHeight = Units.inchesToMeters(57.13);
    } else if (tagId == 11
        || tagId == 12
        || tagId == 13
        || tagId == 14
        || tagId == 15
        || tagId == 16) {
      targetHeight = Units.inchesToMeters(52.00);
    } else {
      System.out.println("Id para AprilTag Inválida");
    }
    ;
    return targetHeight;
  }
}
