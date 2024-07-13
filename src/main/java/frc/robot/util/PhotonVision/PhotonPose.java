package frc.robot.util.PhotonVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.UtilConstants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonPose {

  public Pose2d getRobotPose() {
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
    return robotPose2d;
  }
}
