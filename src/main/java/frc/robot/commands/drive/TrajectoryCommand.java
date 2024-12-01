package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;
import java.util.Arrays;
import java.util.List;

public class TrajectoryCommand extends Command {
  private final LocalADStarAK pathfinder = new LocalADStarAK();
  private final Translation2d startPose;
  private final Translation2d endPose;
  private PathPlannerPath currentPath;
  private Translation2d goalTranslation = new Translation2d(5, 5);
  private PathConstraints constraints = new PathConstraints(14.5, 5, 6, 6);
  private Rotation2d rotationGoalEndState = new Rotation2d(0, 0);
  private GoalEndState goalEndState = new GoalEndState(0, rotationGoalEndState);
  private Command pathfollower;

  // Lista de obstáculos (exemplo)
  private final List<Translation2d> obstacles =
      Arrays.asList(new Translation2d(2, 2), new Translation2d(5, 5), new Translation2d(6, 1));

  public TrajectoryCommand(Drive drive) {
    hasRequirement(drive);
    this.startPose = drive.getPose().getTranslation();
    this.endPose = goalTranslation;
  }

  @Override
  public void initialize() {
    pathfinder.setStartPosition(startPose);
    pathfinder.setGoalPosition(endPose);

    for (Translation2d obstacle : obstacles) {
      pathfinder.setDynamicObstacles(List.of(new Pair<>(obstacle, obstacle)), startPose);
    }

    currentPath = pathfinder.getCurrentPath(constraints, goalEndState);
    SmartDashboard.putBoolean("Current Path", currentPath != null);
    if (currentPath != null) {
      pathfollower = AutoBuilder.followPath(currentPath);
      SmartDashboard.putBoolean("pathfollower", pathfollower != null);
      pathfollower.initialize();
    }
  }

  @Override
  public void execute() {
    if (currentPath != null) {
      pathfollower.execute();
    }

    SmartDashboard.putBoolean("Path Follower is not null?", pathfollower != null);
    if (pathfollower != null) {
      SmartDashboard.putBoolean("Is finished path?", pathfollower.isFinished());
    }
  }

  @Override
  public boolean isFinished() {
    return pathfollower == null;
  }

  @Override
  public void end(boolean interrupted) {
    // drive.stop();
  }
}
