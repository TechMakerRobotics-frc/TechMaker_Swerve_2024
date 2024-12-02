package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class UpdatePoseCommand extends Command {

  private Drive drive;
  private VisionPose visionPose;
  private Pose3d robotPosefromFLCam;
  private Pose3d robotPosefromFRCam;
  private Pose3d robotPosefromLimelight;

  public UpdatePoseCommand(Drive drive, VisionPose visionPose) {
    this.drive = drive;
    this.visionPose = visionPose;
    addRequirements(visionPose);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    robotPosefromLimelight = visionPose.getEstimatedGlobalPoseLimelight().get().estimatedPose;

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
      drive.addVisionMeasurement(
          robotPosefromLimelight.toPose2d(),
          visionPose.getTargetManager().getCamera().getLatestResult().getTimestampSeconds());
    }
  }
}
