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
import org.photonvision.PhotonUtils;

public class AlingCommand extends Command {

  PhotonTags vision;
  private double distanceGoal;
  private int tag;
  private Drive drive;
  private PhotonTags photonTags;
  private static double CAMERA_PITCH = Units.degreesToRadians(30);
  private static double CAMERA_HEIGHT_METERS = 0.20;
  IdTargetHeight targetHeight;

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  final double P_GAIN = 0.1;
  final double D_GAIN = 0.0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  public AlingCommand(double distanceGoal, int tag, Drive drive) {
    this.distanceGoal = distanceGoal;
    this.tag = tag;
    this.drive = drive;
    photonTags = new PhotonTags();
  }

  @Override
  public void execute() {
    aling();
    go();
  }

  private void aling() {
    double rotationSpeed;
    if (photonTags.hasTarget()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(photonTags.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0;
    }
    boolean isFlipped =
        DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
    Rotation2d fieldRelativeRotation =
        isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, rotationSpeed * drive.getMaxAngularSpeedRadPerSec(), fieldRelativeRotation);

    drive.runVelocity(fieldRelativeSpeeds);
  }

  public void go() {
    double forwardSpeed;
    if (photonTags.hasTarget()) {
      // First calculate range
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              targetHeight.idToHeight(1),
              CAMERA_PITCH,
              Units.degreesToRadians(photonTags.getBestTarget().getPitch()));

      forwardSpeed = range * drive.getMaxLinearSpeedMetersPerSec();
      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -controller.calculate(range, distanceGoal);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
  }

  /**
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
