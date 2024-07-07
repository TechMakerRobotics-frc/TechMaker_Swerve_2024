package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.*;
import frc.robot.subsystems.drive.Drive;

public class MoveXYHeading extends Command {
  private final Drive drive;
  private final double targetXMeters, targetYMeters, targetHeadingDegrees;

  private final PIDController xController = new PIDController(KMoveX.KP, KMoveX.KI, KMoveX.KD);
  private final PIDController yController = new PIDController(KMoveY.KP, KMoveY.KI, KMoveY.KD);
  private final PIDController headingController =
      new PIDController(KMoveH.KP, KMoveH.KI, KMoveH.KD);

  private double lastTimestamp;
  private boolean finish = false;

  public MoveXYHeading(Drive drive, double xMeters, double yMeters, double headingDegrees) {
    this.drive = drive;
    this.targetXMeters = xMeters;
    this.targetYMeters = yMeters;
    this.targetHeadingDegrees = headingDegrees;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    lastTimestamp = Timer.getFPGATimestamp();
  }

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
        (Math.abs(errorX) < 0.1 && Math.abs(errorY) < 0.1 && Math.abs(errorHeading) < 0.1)
        || temp >= TimeK.TIME_OUT;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}
