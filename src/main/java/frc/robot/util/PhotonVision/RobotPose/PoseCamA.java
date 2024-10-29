package frc.robot.util.PhotonVision.RobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhotonVision.PhotonTags;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class PoseCamA extends SubsystemBase {

  private static final PhotonCamera cam = PhotonTags.getCameraA();
  private Transform3d fieldToCamera;
  private static Pose3d pose3d;

  @Override
  public void periodic() {
    PhotonPipelineResult result = cam.getLatestResult();
    if (result.hasTargets()) {
      
    } else {
      fieldToCamera = null;
    }
    SmartDashboard.putBoolean("Has Target A", result.hasTargets());
    SmartDashboard.putBoolean("Field to camera A", fieldToCamera != null);
    if (fieldToCamera != null) {
      pose3d = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
      Logger.recordOutput("Field To Target A", pose3d);
      Field2d field = new Field2d();
      field.setRobotPose(pose3d.toPose2d());
      SmartDashboard.putData("Camera A", field);
    } else {
      pose3d = null;
    }
  }

  public static Pose3d getPose3d() {
    return pose3d;
  }
}
