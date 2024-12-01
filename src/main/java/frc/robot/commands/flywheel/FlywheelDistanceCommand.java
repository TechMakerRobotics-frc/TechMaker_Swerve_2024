package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelSpeedMap;
import frc.robot.vision.VisionManager;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** Command to align the robot using vision targets detected by PhotonVision. */
public class FlywheelDistanceCommand extends Command {

  private final VisionManager manager;
  private final FlywheelSpeedMap speedMap = new FlywheelSpeedMap();
  private final Flywheel flywheel;

  public FlywheelDistanceCommand(PhotonCamera limelight, Flywheel flywheel) {
    this.flywheel = flywheel;
    manager = new VisionManager(limelight);
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = manager.getLatestPipeline();
    if (manager.hasTarget(p)) {
      double speed =
          speedMap.CalculateFlywheelSpeed(manager.getBestCamera(manager.getBestTarget(p)).getX());
      SmartDashboard.putNumber(
          "Distance to tag - Flywheel Distance",
          manager.getBestCamera(manager.getBestTarget(p)).getX());
      flywheel.runVelocity(speed);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }
}
