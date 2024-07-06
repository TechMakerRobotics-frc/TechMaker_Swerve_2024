package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.IdTargetHeight;
import frc.robot.util.PhotonTags;
import frc.robot.util.UtilConstants.VisionConstants;

import org.photonvision.PhotonUtils;

public class AlingCommand extends Command {

  private final PhotonTags photonTags;
  private final double distanceGoal;
  private final int tag;
  private final Drive drive;
  private IdTargetHeight targetHeight;

  // PID constants should be tuned per robot
  private static final double LINEAR_P = 0.1;
  private static final double LINEAR_D = 0.0;
  private final PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  private static final double ANGULAR_P = 0.1;
  private static final double ANGULAR_D = 0.0;
  private final PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public AlingCommand(double distanceGoal, int tag, Drive drive) {
    this.distanceGoal = distanceGoal;
    this.tag = tag;
    this.drive = drive;
    this.photonTags = new PhotonTags();
    addRequirements(drive);
  }

  @Override
  public void execute() {
    align();
    go();
  }

  private void align() {
    double rotationSpeed = 0;
    if (photonTags.hasTag()) {
      rotationSpeed = -turnController.calculate(photonTags.getBestTarget().getYaw(), 0);
      boolean isFlipped =
          DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
      Rotation2d fieldRelativeRotation =
          isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
      ChassisSpeeds fieldRelativeSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0, 0, rotationSpeed * drive.getMaxAngularSpeedRadPerSec(), fieldRelativeRotation);
      drive.runVelocity(fieldRelativeSpeeds);
    }
  }

  private void go() {
    double forwardSpeed = 0;
    if (photonTags.hasTag()) {
      // First calculate range
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              VisionConstants.CAMERA_HEIGHT_METERS,
              targetHeight.idToHeight(tag),
              VisionConstants.CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(photonTags.getBestTarget().getPitch()));
      // Use this range as the measurement we give to the PID controller.
      forwardSpeed = -forwardController.calculate(range, distanceGoal);
      forwardSpeed =
          Math.max(
              -drive.getMaxLinearSpeedMetersPerSec(),
              Math.min(forwardSpeed, drive.getMaxLinearSpeedMetersPerSec()));
    }
    drive.runVelocity(new ChassisSpeeds(forwardSpeed, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
