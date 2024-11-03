package frc.robot.util.PhotonVision;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionPose extends SubsystemBase {

  private static final PhotonCamera FLcam = VisionTagsFLCam.getCamera();
  private static final PhotonCamera FRcam = VisionTagsFRCam.getCamera();
  private static final PhotonCamera limelight = VisionTagsLimelight.getCamera();

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private Transform3d robotToFLCam =
      new Transform3d(new Translation3d(0.5, -0.25, 0.25), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorFLCam =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, FLcam, robotToFLCam);

  private Transform3d robotToFRCam =
      new Transform3d(new Translation3d(0.5, 0.25, 0.25), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorFRCam =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, FRcam, robotToFRCam);

  private Transform3d robotToLimelight =
      new Transform3d(new Translation3d(0.2, 0.0, 0.30), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorLimelight =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, limelight, robotToLimelight);

  private Drive drive;

  private Pose2d pose2dFLCam;
  private Pose2d pose2dFRCam;
  private Pose2d pose2dLimelight;

  private Pose3d pose3dFLCam;
  private Pose3d pose3dFRCam;
  private Pose3d pose3dLimelight;

  private Pose2d combinedPose2d;
  private Pose3d combinedPose3d;

  private Field2d fieldFLCam = new Field2d();
  private Field2d fieldFRCam = new Field2d();
  private Field2d fieldLimelight = new Field2d();
  private Field2d fieldCombinedTags = new Field2d();
  private Field2d fieldCombinedTagsPose3d = new Field2d();

  private double xSum = 0.0;
  private double ySum = 0.0;
  private double zSum = 0.0;

  private double thetaSum = 0.0;
  private double thetaXSum = 0.0;
  private double thetaYSum = 0.0;
  private double thetaZSum = 0.0;

  private int count = 0;

  public VisionPose(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPoseOptFLCam =
        getEstimatedGlobalPoseFLCam(drive.getPose());
    Optional<EstimatedRobotPose> estimatedPoseOptFRCam =
        getEstimatedGlobalPoseFRCam(drive.getPose());
    Optional<EstimatedRobotPose> estimatedPoseOptLimelight =
        getEstimatedGlobalPoseLimelight(drive.getPose());

    xSum = 0.0;
    ySum = 0.0;
    zSum = 0.0;

    thetaSum = 0.0;
    thetaXSum = 0.0;
    thetaYSum = 0.0;
    thetaZSum = 0.0;

    count = 0;

    if (estimatedPoseOptFLCam.isPresent()) {
      pose2dFLCam = estimatedPoseOptFLCam.get().estimatedPose.toPose2d();
      pose3dFLCam = estimatedPoseOptFLCam.get().estimatedPose;
      fieldFLCam.setRobotPose(pose2dFLCam);
      SmartDashboard.putData("Field to cam FL", fieldFLCam);

      xSum += pose2dFLCam.getX();
      ySum += pose2dFLCam.getY();
      zSum += pose3dFLCam.getZ();

      thetaSum += pose2dFLCam.getRotation().getRadians();
      thetaXSum += pose3dFLCam.getRotation().getX();
      thetaYSum += pose3dFLCam.getRotation().getY();
      thetaZSum += pose3dFLCam.getRotation().getZ();

      count++;
    }

    if (estimatedPoseOptFRCam.isPresent()) {
      pose2dFRCam = estimatedPoseOptFRCam.get().estimatedPose.toPose2d();
      pose3dFRCam = estimatedPoseOptFRCam.get().estimatedPose;
      fieldFRCam.setRobotPose(pose2dFRCam);
      SmartDashboard.putData("Field to cam FR", fieldFRCam);

      xSum += pose2dFRCam.getX();
      ySum += pose2dFRCam.getY();
      zSum += pose3dFRCam.getZ();

      thetaSum += pose2dFRCam.getRotation().getRadians();
      thetaXSum += pose3dFRCam.getRotation().getX();
      thetaYSum += pose3dFRCam.getRotation().getY();
      thetaZSum += pose3dFRCam.getRotation().getZ();

      count++;
    }

    if (estimatedPoseOptLimelight.isPresent()) {
      pose2dLimelight = estimatedPoseOptLimelight.get().estimatedPose.toPose2d();
      pose3dLimelight = estimatedPoseOptLimelight.get().estimatedPose;

      fieldLimelight.setRobotPose(pose2dLimelight);
      SmartDashboard.putData("Field to cam Limelight", fieldLimelight);

      xSum += pose2dLimelight.getX();
      ySum += pose2dLimelight.getY();
      zSum += pose3dLimelight.getZ();

      thetaSum += pose2dLimelight.getRotation().getRadians();
      thetaXSum += pose3dLimelight.getRotation().getX();
      thetaYSum += pose3dLimelight.getRotation().getY();
      thetaZSum += pose3dLimelight.getRotation().getZ();

      count++;
    }

    if (count > 0) {
      combinedPose2d = new Pose2d(xSum / count, ySum / count, new Rotation2d(thetaSum / count));
      fieldCombinedTags.setRobotPose(combinedPose2d);
      SmartDashboard.putData("Estimated Combined Pose", fieldCombinedTags);
    }

    if (count > 0) {
      combinedPose3d =
          new Pose3d(
              new Translation3d(xSum / count, ySum / count, zSum / count),
              new Rotation3d(thetaXSum / count, thetaYSum / count, thetaZSum / count));
      fieldCombinedTagsPose3d.setRobotPose(combinedPose3d.toPose2d());
      SmartDashboard.putData(
          "Estimated Combined Pose3d to pose 2d to test", fieldCombinedTagsPose3d);
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

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLimelight(
      Pose2d prevEstimatedRobotPose) {
    photonPoseEstimatorLimelight.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimatorLimelight.update();
  }

  /** Retorna a pose3d pela câmera FL */
  @AutoLogOutput(key = "Odometry/FLCamPose3d")
  public Pose3d getRobotPoseFLCam() {
    Optional<EstimatedRobotPose> estimatedPoseOptFLCam =
        getEstimatedGlobalPoseFLCam(drive.getPose());
    if (estimatedPoseOptFLCam.isPresent()) {
      return pose3dFLCam;
    } else {
      return null;
    }
  }

  /** Retorna a pose3d pela câmera FR */
  @AutoLogOutput(key = "Odometry/FRCamPose3d")
  public Pose3d getRobotPoseFRCam() {
    Optional<EstimatedRobotPose> estimatedPoseOptFRCam =
        getEstimatedGlobalPoseFRCam(drive.getPose());
    if (estimatedPoseOptFRCam.isPresent()) {
      return pose3dFRCam;
    } else {
      return null;
    }
  }

  /** Retorna a pose3d pela câmera limelight */
  @AutoLogOutput(key = "Odometry/LimelightPose3d")
  public Pose3d getRobotPoseLimelight() {
    Optional<EstimatedRobotPose> estimatedPoseOptLimelight =
        getEstimatedGlobalPoseLimelight(drive.getPose());
    if (estimatedPoseOptLimelight.isPresent()) {
      return pose3dLimelight;
    } else {
      return null;
    }
  }

  /** Retorna a pose combinada das câmeras */
  @AutoLogOutput(key = "Odometry/CombinedPose3d")
  public Pose3d getRobotPose3d() {
    if (count > 0) {
      return combinedPose3d;
    } else {
      return null;
    }
  }
}

/*
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
  private static final PhotonCamera limelight = VisionTagsLimelight.getCamera();

  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private Transform3d robotToFLCam =
      new Transform3d(new Translation3d(0.5, -0.25, 0.25), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorFLCam =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, FLcam, robotToFLCam);

  private Transform3d robotToFRCam =
      new Transform3d(new Translation3d(0.5, 0.25, 0.25), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorFRCam =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, FRcam, robotToFRCam);

  private Transform3d robotToLimelight =
      new Transform3d(new Translation3d(0.2, 0.0, 0.30), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimatorLimelight =
      new PhotonPoseEstimator(
          aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, limelight, robotToLimelight);

  private Drive drive;

  private Pose2d pose2dFLCam;
  private Pose2d pose2dFRCam;
  private Pose2d pose2dLimelight;

  private Field2d fieldFLCam = new Field2d();
  private Field2d fieldFRCam = new Field2d();
  private Field2d fieldLimelight = new Field2d();

  public VisionPose(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPoseOptFLCam =
        getEstimatedGlobalPoseFLCam(drive.getPose());

    Optional<EstimatedRobotPose> estimatedPoseOptFRCam =
        getEstimatedGlobalPoseFRCam(drive.getPose());

    Optional<EstimatedRobotPose> estimatedPoseOptLimelight =
        getEstimatedGlobalPoseLimelight(drive.getPose());

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

    if (estimatedPoseOptLimelight.isPresent()) {
      pose2dLimelight = estimatedPoseOptLimelight.get().estimatedPose.toPose2d();
      fieldLimelight.setRobotPose(pose2dLimelight);

      SmartDashboard.putData("Field to cam Limelight", fieldLimelight);
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

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLimelight(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimatorLimelight.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimatorLimelight.update();
  }
}
 */
