package frc.robot.util.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonPose extends SubsystemBase {

    private final Field2d field = new Field2d();
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private static final PhotonCamera cam = PhotonTags.getCamera();
    private static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    private static final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

    public PhotonPose() {
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = cam.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            int fiducialId = bestTarget.getFiducialId();
            Optional<Pose3d> fieldTagPose = aprilTagFieldLayout.getTagPose(fiducialId);

            if (fieldTagPose.isPresent()) {
                Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
                Pose2d estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, fieldTagPose.get(), robotToCam).toPose2d();
                field.setRobotPose(estimatedPose);
                SmartDashboard.putData("Field robot pose PhotonVision", field);
            }
        }

        SmartDashboard.putBoolean("Has Target", result.hasTargets());
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
