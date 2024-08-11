package frc.robot.util.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.UtilConstants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonPose extends SubsystemBase {

  Field2d field;

  // The field from AprilTagFields will be different depending on the game.
  static AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Forward Camera
  static PhotonCamera cam = PhotonTags.getCamera();

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  static Transform3d robotToCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  // Construct PhotonPoseEstimator
  static PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

  public PhotonPose() {}

  @Override
  public void periodic(){
    PhotonPipelineResult p = PhotonTags.getLatestPipeline();
    
    if (PhotonTags.hasTarget(p)) {
      PhotonCamera camera = PhotonTags.getCamera();
      PhotonTrackedTarget bestTarget = camera.getLatestResult().getBestTarget();

      Transform2d cameraToTarget =
          new Transform2d(
              new Translation2d(
                  bestTarget.getBestCameraToTarget().getX(),
                  bestTarget.getBestCameraToTarget().getY()),
              Rotation2d.fromDegrees(bestTarget.getYaw()));

      Pose2d fieldToTarget =
          new Pose2d(
              VisionConstants.TARGET_7_X_POSITION_METERS,
              VisionConstants.TARGET_7_Y_POSITION_METERS,
              Rotation2d.fromDegrees(VisionConstants.TARGET_7_YAW_DEGREES));

      // Construct cameraToRobot
      Transform2d cameraToRobot =
          new Transform2d(
              new Translation2d(
                  VisionConstants.CAMERA_POSITION_FROM_ROBOT_CENTER_X_METERS,
                  VisionConstants.CAMERA_POSITION_FROM_ROBOT_CENTER_Y_METERS),
              Rotation2d.fromDegrees(0));

      Pose2d robotPose2d =
          PhotonUtils.estimateFieldToRobot(cameraToTarget, fieldToTarget, cameraToRobot);
      field.setRobotPose(robotPose2d);
      SmartDashboard.putData("Field robot pose PhotonVision", field);
    }
  }
  public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
