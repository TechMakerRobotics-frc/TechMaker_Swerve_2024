package frc.robot.util.PhotonVision.RobotPose;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RobotPose extends SubsystemBase {

  private final Pose3d poseA = PoseCamA.getPose3d();
  private final Pose3d poseB = PoseCamB.getPose3d();

  @Override
  public void periodic() {
    if (poseA != null && poseB != null) {
      Translation3d averageTranslation = new Translation3d(
        (poseA.getX() + poseB.getX()) / 2,
        (poseA.getY() + poseB.getY()) / 2,
        (poseA.getZ() + poseB.getZ()) / 2
      );

      Rotation3d averageRotation = new Rotation3d(
        (poseA.getRotation().getX() + poseB.getRotation().getX()) / 2,
        (poseA.getRotation().getY() + poseB.getRotation().getY()) / 2,
        (poseA.getRotation().getZ() + poseB.getRotation().getZ()) / 2
      );

      Pose3d estimatedPose = new Pose3d(averageTranslation, averageRotation);

      Logger.recordOutput("Field To Target - Test 2 Cam", estimatedPose);
    }
  }
}
