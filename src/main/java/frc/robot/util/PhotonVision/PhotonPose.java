package frc.robot.util.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIOPigeon2;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonPose extends SubsystemBase {

    private GyroIOPigeon2 pigeon = new GyroIOPigeon2();
    private final Field2d field = new Field2d();
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private static final PhotonCamera cam = PhotonTags.getCamera();
    private static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d());
    private static final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);

    @Override
    public void periodic() {
        PhotonPipelineResult result = cam.getLatestResult();
        if (result.hasTargets()) {
            double xPose = PhotonTags.getBestCamera(result.getBestTarget()).getX();
            double yPose = PhotonTags.getBestCamera(result.getBestTarget()).getY();
            double zPose = PhotonTags.getBestCamera(result.getBestTarget()).getZ();
            new Pose3d(xPose, yPose, zPose, pigeon.getRotation3d());
        }
        SmartDashboard.putBoolean("Has Target", result.hasTargets());
    }
}
