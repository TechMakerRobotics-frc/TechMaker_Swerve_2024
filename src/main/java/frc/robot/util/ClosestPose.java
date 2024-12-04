package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A utility class for finding the closest pose to a given pose.
 *
 * <p>This class provides a method to determine the nearest {@link Pose2d} from a collection of
 * candidate poses relative to the current pose of the robot.
 */
public class ClosestPose {

  /**
   * Finds the closest pose to the given current pose from a collection of poses.
   *
   * @param currentPose The current pose of the robot.
   * @param poses An {@link Iterable} of candidate poses to compare.
   * @return The closest pose to the current pose, or null if the input list is empty.
   */
  public static Pose2d findClosestPose(Pose2d currentPose, Iterable<Pose2d> poses) {
    Pose2d bestPose = null;
    double bestDistance = Double.MAX_VALUE;

    // Iterate through all candidate poses to find the closest one
    for (Pose2d pose : poses) {
      double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
      if (distance < bestDistance) {
        bestDistance = distance;
        bestPose = pose;
      }
    }

    return bestPose;
  }
}
