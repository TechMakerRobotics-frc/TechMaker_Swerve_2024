package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandConstants.*;
import frc.robot.subsystems.drive.Drive;

/** Command to move the robot to a specific position (X, Y) and heading using PID controllers. */
public class MoveXYHeading extends Command {
  private final Drive drive;
  private double targetXMeters, targetYMeters, targetHeadingDegrees;

  private final PIDController xController = new PIDController(KMoveX.KP, KMoveX.KI, KMoveX.KD);
  private final PIDController yController = new PIDController(KMoveY.KP, KMoveY.KI, KMoveY.KD);
  private final PIDController headingController =
      new PIDController(KMoveH.KP, KMoveH.KI, KMoveH.KD);

  private double lastTimestamp,
      xVelocity,
      yVelocity,
      angVelocity,
      errorX,
      errorY,
      errorHeading;
  private boolean finish = false;

  private Pose2d currentPose;

  /**
   * Constructs a new MoveXYHeading command.
   *
   * @param drive The drive subsystem used by this command.
   * @param xMeters The target position in meters on the X axis.
   * @param yMeters The target position in meters on the Y axis.
   * @param headingDegrees The target heading in degrees.
   */
  public MoveXYHeading(Drive drive, double xMeters, double yMeters, double headingDegrees) {
    this.drive = drive;
    this.targetXMeters = xMeters;
    this.targetYMeters = yMeters;
    this.targetHeadingDegrees = headingDegrees;
    addRequirements(drive);
  }

  /** Initializes the command by resetting the last timestamp. */
  @Override
  public void initialize() {
    lastTimestamp = Timer.getFPGATimestamp();

    Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive);
  }

  /**
   * Executes the command, calculating the necessary velocities using PID controllers and applying
   * them to the drive subsystem.
   */
  @Override
  public void execute() {
    double currentHeading = drive.getRotation().getDegrees();

    while (finish == false) {
      currentPose = drive.getPose();
      errorX = targetXMeters - currentPose.getX();
      errorY = targetYMeters - currentPose.getY();
      errorHeading = targetHeadingDegrees - currentHeading;
    }

    if (errorX >= 0.08 && errorY >= 0.08) {
      xVelocity = xController.calculate(currentPose.getX(), errorX);
      yVelocity = yController.calculate(currentPose.getY(), errorY);
      angVelocity = headingController.calculate(currentHeading, errorHeading);
    } else {
      drive.stop();
      finish = true;
    }
    double temp = Timer.getFPGATimestamp() - lastTimestamp;

    drive.runVelocity(new ChassisSpeeds(xVelocity, yVelocity, Math.toRadians(angVelocity)));

    finish =
        (Math.abs(errorX) < 0.1 && Math.abs(errorY) < 0.1 && Math.abs(errorHeading) < 0.1)
        || temp >= TimeK.TIME_OUT;
  }

  /**
   * Ends the command, stopping the drive subsystem.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  /**
   * Determines whether the command has finished.
   *
   * @return true if the command has finished, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return finish;
  }
}
