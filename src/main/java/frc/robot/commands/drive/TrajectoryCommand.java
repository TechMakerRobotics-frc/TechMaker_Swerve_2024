package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TrajectoryCommand extends Command {
  private final Pose2d goalPose;
  private final double timeOut;
  private PathConstraints constraints = new PathConstraints(14.5, 5, 6, 6);
  private Command pathfollower;
  private Timer time = new Timer();

  public TrajectoryCommand(Pose2d goalPose, double timeOut) {
    this.timeOut = timeOut;
    this.goalPose = goalPose;
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
    pathfollower = AutoBuilder.pathfindToPose(goalPose, constraints);
    pathfollower.initialize();
  }

  @Override
  public void execute() {
    pathfollower.execute();
  }

  @Override
  public boolean isFinished() {
    return pathfollower == null || time.hasElapsed(timeOut);
  }
}
