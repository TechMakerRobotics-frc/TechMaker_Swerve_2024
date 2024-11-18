package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.*;

public class VisionSim extends SubsystemBase {
  /** The drive subsystem used by this class. */
  private final Drive drive;

  /** A vision system sim labelled as "main" in NetworkTables. */
  private static VisionSystemSim visionSim = new VisionSystemSim("main");

  /** A 0.5 x 0.25 meter rectangular target. */
  private TargetModel targetModel = new TargetModel(0.5, 0.25);

  /**
   * The pose of where the target is on the field. Its rotation determines where "forward" or the
   * target x-axis points. Let's say this target is flat against the far wall center, facing the
   * blue driver stations.
   */
  private Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));

  /** The given target model at the given pose. */
  private VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

  /*
   *The layout of AprilTags which we want to add to the vision system.
   */
  private AprilTagFieldLayout tagLayout;

  /** The simulated camera properties. */
  private SimCameraProperties flCamProp = new SimCameraProperties();
  private SimCameraProperties frCamProp = new SimCameraProperties();
  private SimCameraProperties limelightProp = new SimCameraProperties();

  private final VisionManager managerFL;
  private final VisionManager managerFR;
  private final VisionManager managerLimelight;

  private final PhotonCameraSim cameraFLSim;
  private final PhotonCameraSim cameraFRSim;
  private final PhotonCameraSim limelightSim;

  private final Transform3d robotToFLCam;
  private final Transform3d robotToFRCam;
  private final Transform3d robotToLimelight;

  public VisionSim(Drive drive, PhotonCamera FLCam, PhotonCamera FRCam, PhotonCamera limelight) {
    managerFL = new VisionManager(FLCam);
    managerFR = new VisionManager(FRCam);
    managerLimelight = new VisionManager(limelight);

    cameraFLSim = new PhotonCameraSim(managerFL.getCamera(), flCamProp);
    cameraFRSim = new PhotonCameraSim(managerFR.getCamera(), frCamProp);
    limelightSim = new PhotonCameraSim(managerLimelight.getCamera(), limelightProp);

    robotToFLCam = managerFL.getCameraTransform3d();
    robotToFRCam = managerFR.getCameraTransform3d();
    robotToLimelight = managerLimelight.getCameraTransform3d();

    this.drive = drive;
    try {
      // Carrega o layout dos AprilTags a partir do recurso especificado
      tagLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      // Adiciona este alvo de visão à simulação do sistema de visão para torná-lo visível
      visionSim.addVisionTargets(visionTarget);
      visionSim.addAprilTags(tagLayout);
    } catch (IOException e) {
      // Tratamento da exceção
      e.printStackTrace();
    }

    // Add this vision target to the vision system simulation to make it visible
    visionSim.addVisionTargets(visionTarget);
    visionSim.addAprilTags(tagLayout);

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    flCamProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    frCamProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    limelightProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));

    // Approximate detection noise with average and standard deviation error in pixels.
    flCamProp.setCalibError(0.25, 0.08);
    frCamProp.setCalibError(0.25, 0.08);
    limelightProp.setCalibError(0.25, 0.08);

    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    flCamProp.setFPS(20);
    frCamProp.setFPS(20);
    limelightProp.setFPS(20);

    // The average and standard deviation in milliseconds of image data latency.
    flCamProp.setAvgLatencyMs(35);
    flCamProp.setLatencyStdDevMs(5);

    frCamProp.setAvgLatencyMs(35);
    frCamProp.setLatencyStdDevMs(5);

    limelightProp.setAvgLatencyMs(35);
    limelightProp.setLatencyStdDevMs(5);

    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    visionSim.addCamera(cameraFLSim, robotToFLCam);
    visionSim.addCamera(cameraFRSim, robotToFRCam);
    visionSim.addCamera(limelightSim, robotToLimelight);
  }

  @Override
  public void periodic() {
    // Update with the simulated drivetrain pose. This should be called every loop in simulation.
    visionSim.update(drive.getPose());
    visionSim.getDebugField();
  }

  /** Get the built-in Field2d used by this VisionSystemSim. */
  public static Field2d getField2dVisionSim() {
    return visionSim.getDebugField();
  }
}
