package frc.robot.vision;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.VisionConstants.CameraConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * A classe VisionPose representa o subsistema de visão utilizado para estimar a posição e a
 * orientação do robô usando câmeras PhotonVision e AprilTags.
 */
public class VisionPose extends SubsystemBase implements VisionPoseIO {

  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private final PhotonPoseEstimator photonPoseEstimatorFLCam;
  private final PhotonPoseEstimator photonPoseEstimatorFRCam;
  private final PhotonPoseEstimator photonPoseEstimatorLimelight;

  private final VisionManager flManager;
  private final VisionManager frManager;
  private final VisionManager limelightManager;
  private final VisionManager centralManager;

  private final PhotonCamera flCam = new PhotonCamera(CameraConstants.CAMERA_FL_NAME);
  private final PhotonCamera frCam = new PhotonCamera(CameraConstants.CAMERA_FR_NAME);
  private final PhotonCamera limelight = new PhotonCamera(CameraConstants.LIMELIGHT_NAME);
  private final PhotonCamera centralCam = new PhotonCamera(CameraConstants.CENTRAL_CAM_NAME);

  /** Construtor da classe VisionPose. * */
  public VisionPose() {
    flManager = new VisionManager(flCam);
    frManager = new VisionManager(frCam);
    limelightManager = new VisionManager(limelight);
    centralManager = new VisionManager(flCam);

    photonPoseEstimatorFLCam =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            flManager.getCamera(),
            flManager.getCameraTransform3d());

    photonPoseEstimatorFRCam =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frManager.getCamera(),
            frManager.getCameraTransform3d());

    photonPoseEstimatorLimelight =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            limelightManager.getCamera(),
            frManager.getCameraTransform3d());
  }

  @Override
  public void periodic() {}

  /** Obtém a pose estimada global usando a câmera FL. */
  @AutoLogOutput(key = "Odometry/FLCamPose3d")
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFLCam() {
    return photonPoseEstimatorFLCam.update();
  }

  /** Obtém a pose estimada global usando a câmera FR. */
  @AutoLogOutput(key = "Odometry/FRCamPose3d")
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFRCam() {
    return photonPoseEstimatorFRCam.update();
  }

  /** Obtém a pose estimada global usando a câmera Limelight. */
  @AutoLogOutput(key = "Odometry/LimelightPose3d")
  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLimelight() {
    return photonPoseEstimatorLimelight.update();
  }

  public VisionManager getFLManager() {
    return flManager;
  }

  public VisionManager getFRManager() {
    return frManager;
  }

  public VisionManager getTargetManager() {
    return limelightManager;
  }

  public VisionManager getCentralManager() {
    return centralManager;
  }
}
