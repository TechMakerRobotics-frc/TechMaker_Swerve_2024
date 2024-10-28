package frc.robot.util.PhotonVision.RobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhotonVision.PhotonTags;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PoseCamB extends SubsystemBase {

  private static final PhotonCamera cam = PhotonTags.getCameraB();
  private Transform3d fieldToCamera;
  private static Pose3d pose3d;

  @Override
  public void periodic() {
    PhotonPipelineResult result = cam.getLatestResult();
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      fieldToCamera = result.getMultiTagResult().estimatedPose.best;
    } else {
      fieldToCamera = null;
    }
    SmartDashboard.putBoolean("Has Target", result.hasTargets());
    if (fieldToCamera != null) {
      pose3d = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
      Logger.recordOutput("Field To Target", pose3d);
    } else {
      pose3d = null;
    }
  }

  public static Pose3d getPose3d() {
    return pose3d;
  }
}
