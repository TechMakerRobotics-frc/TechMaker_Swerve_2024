package frc.robot.util.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionPose extends SubsystemBase {

  private static final PhotonCamera cam = VisionTagsCamA.getCamera();
  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private Transform3d robotToCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

  Drive drive;

  public VisionPose(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPoseOpt = getEstimatedGlobalPose(drive.getPose());

    if (estimatedPoseOpt.isPresent()) {
      Pose2d pose2d = estimatedPoseOpt.get().estimatedPose.toPose2d();
      Field2d field = new Field2d();
      field.setRobotPose(pose2d);

      SmartDashboard.putData(field);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
