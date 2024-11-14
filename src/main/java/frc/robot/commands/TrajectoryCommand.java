package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;

public class TrajectoryCommand extends Command {
  private final LocalADStarAK pathfinder = new LocalADStarAK();
  private final Translation2d startPose;
  private final Translation2d endPose;
  private PathPlannerPath currentPath;
  private Translation2d goalTranslation = new Translation2d(5, 5);
  private Drive drive;
  private PathConstraints constraints = new PathConstraints(2, 5, 6, 6);
  private Rotation2d rotationGoalEndState = new Rotation2d(0);
  private GoalEndState goalEndState = new GoalEndState(0, rotationGoalEndState);
  private Command pathfollower;

  public TrajectoryCommand(Drive drive) {
    hasRequirement(drive);
    this.drive = drive;
    this.startPose = drive.getPose().getTranslation();
    this.endPose = goalTranslation;
  }

  @Override
  public void initialize() {
    pathfinder.setStartPosition(startPose);
    pathfinder.setGoalPosition(endPose);
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
  }

  @Override
  public boolean isFinished() {
    return (pathfollower == null) || (pathfollower.isFinished());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
