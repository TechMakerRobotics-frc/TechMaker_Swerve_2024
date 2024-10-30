package frc.robot.util.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.Optional;

public class VisionPose extends SubsystemBase {

  private static final PhotonCamera cam = PhotonTagsCamA.getCamera();
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  // Construct PhotonPoseEstimator
  private PhotonPoseEstimator photonPoseEstimator = 
    new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);

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