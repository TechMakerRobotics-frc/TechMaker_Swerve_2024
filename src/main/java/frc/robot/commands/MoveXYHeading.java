package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.*;
import frc.robot.subsystems.drive.Drive;

/**
 * Command to move the robot to a specific position (X, Y) and heading using PID controllers.
 */
public class MoveXYHeading extends Command {
  private final Drive drive;
  private final double targetXMeters, targetYMeters, targetHeadingDegrees;

  private final PIDController xController = new PIDController(KMoveX.KP, KMoveX.KI, KMoveX.KD);
  private final PIDController yController = new PIDController(KMoveY.KP, KMoveY.KI, KMoveY.KD);
  private final PIDController headingController =
      new PIDController(KMoveH.KP, KMoveH.KI, KMoveH.KD);

  private double lastTimestamp;
  private boolean finish = false;

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

  /**
   * Initializes the command by resetting the last timestamp.
   */
  @Override
  public void initialize() {
    lastTimestamp = Timer.getFPGATimestamp();
  }

  /**
   * Executes the command, calculating the necessary velocities using PID controllers and applying them to the drive subsystem.
   */
  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    double currentHeading = drive.getRotation().getDegrees();

    double errorX = targetXMeters - currentPose.getX();
    double errorY = targetYMeters - currentPose.getY();
    double errorHeading = targetHeadingDegrees - currentHeading;

    double temp = Timer.getFPGATimestamp() - lastTimestamp;

    double xVelocity = xController.calculate(currentPose.getX(), errorX);
    double yVelocity = yController.calculate(currentPose.getY(), errorY);
    double angVelocity = headingController.calculate(currentHeading, errorHeading);

    drive.runVelocity(new ChassisSpeeds(xVelocity, yVelocity, Math.toRadians(angVelocity)));

    finish =
        (Math.abs(xVelocity) < 0.1 && Math.abs(yVelocity) < 0.1 && Math.abs(angVelocity) < 0.1)
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
