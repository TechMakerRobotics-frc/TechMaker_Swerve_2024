package frc.robot.util.PhotonVision;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionPose extends SubsystemBase {

  private static final PhotonCamera FLcam = VisionTagsFLCam.getCamera();
  private static final PhotonCamera FRcam = VisionTagsFRCam.getCamera();

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private Transform3d robotToFLCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorFLCam =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, FLcam, robotToFLCam);

  private Transform3d robotToFRCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorFRCam =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, FRcam, robotToFRCam);

  private Drive drive;

  private Pose2d pose2dFLCam;
  private Pose2d pose2dFRCam;

  private Field2d fieldFLCam = new Field2d();
  private Field2d fieldFRCam = new Field2d();

  public VisionPose(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPoseOptFLCam =
        getEstimatedGlobalPoseFLCam(drive.getPose());
    Optional<EstimatedRobotPose> estimatedPoseOptFRCam =
        getEstimatedGlobalPoseFRCam(drive.getPose());

    if (estimatedPoseOptFLCam.isPresent()) {
      pose2dFLCam = estimatedPoseOptFLCam.get().estimatedPose.toPose2d();
      fieldFLCam.setRobotPose(pose2dFLCam);

      SmartDashboard.putData("Field to cam FL", fieldFLCam);
    }

    if (estimatedPoseOptFRCam.isPresent()) {
      pose2dFRCam = estimatedPoseOptFRCam.get().estimatedPose.toPose2d();
      fieldFRCam.setRobotPose(pose2dFRCam);

      SmartDashboard.putData("Field to cam FR", fieldFRCam);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFLCam(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimatorFLCam.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimatorFLCam.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFRCam(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimatorFRCam.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimatorFRCam.update();
  }
}
