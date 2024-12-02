package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class PerpetualPoseCommand extends Command {

  private final Drive drive;
  private final VisionPose visionPose;
  private Pose3d robotPosefromFLCam;
  private Pose3d robotPosefromFRCam;
  private Pose3d robotPosefromLimelight;

  private boolean shouldUpdatePose = true;

  public PerpetualPoseCommand(Drive drive, VisionPose visionPose) {
    this.drive = drive;
    this.visionPose = visionPose;
  }

  @Override
  public void execute() {
    if (!shouldUpdatePose) {
      return;
    }

    if (visionPose.getEstimatedGlobalPoseFLCam().isPresent()) {
      robotPosefromFLCam = visionPose.getEstimatedGlobalPoseFLCam().get().estimatedPose;
      drive.addVisionMeasurement(
          robotPosefromFLCam.toPose2d(),
          visionPose.getFLManager().getCamera().getLatestResult().getTimestampSeconds());
    }

    if (visionPose.getEstimatedGlobalPoseFRCam().isPresent()) {
      robotPosefromFRCam = visionPose.getEstimatedGlobalPoseFRCam().get().estimatedPose;
      drive.addVisionMeasurement(
          robotPosefromFRCam.toPose2d(),
          visionPose.getFRManager().getCamera().getLatestResult().getTimestampSeconds());
    }

    if (visionPose.getEstimatedGlobalPoseLimelight().isPresent()) {
      robotPosefromLimelight = visionPose.getEstimatedGlobalPoseLimelight().get().estimatedPose;
      drive.addVisionMeasurement(
          robotPosefromLimelight.toPose2d(),
          visionPose.getTargetManager().getCamera().getLatestResult().getTimestampSeconds());
    }
  }

  public void setShouldUpdatePose(boolean shouldUpdatePose) {
    this.shouldUpdatePose = shouldUpdatePose;
  }

  public boolean isShouldUpdatePose() {
    return shouldUpdatePose;
  }
}
