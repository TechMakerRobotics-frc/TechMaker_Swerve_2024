package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.*;
import frc.robot.subsystems.drive.Drive;

/** Command to move the robot to a specific position (X, Y) and heading using PID controllers. */
public class MoveXYHeading extends Command {
  private Drive drive;
  private double targetXMeters, targetYMeters, targetHeadingDegrees;

  private final PIDController xController = new PIDController(KMoveX.KP, KMoveX.KI, KMoveX.KD);
  private final PIDController yController = new PIDController(KMoveY.KP, KMoveY.KI, KMoveY.KD);
  private final PIDController headingController =
      new PIDController(KMoveH.KP, KMoveH.KI, KMoveH.KD);

  private double lastTimestamp, xVelocity, yVelocity, angVelocity;
  private boolean finish;
  private Command defaultCommand;

  /**
   * Constructs a new MoveXYHeading command.
   *
   * @param xMeters The target position in meters on the X axis.
   * @param yMeters The target position in meters on the Y axis.
   * @param headingDegrees The target heading in degrees.
   * @param drive The drive subsystem used by this command.
   */
  public MoveXYHeading(double xMeters, double yMeters, double headingDegrees, Drive drive) {
    this.drive = drive;
    this.targetXMeters = xMeters;
    this.targetYMeters = yMeters;
    this.targetHeadingDegrees = headingDegrees;
    addRequirements(drive);
  }

  /** Contructs a new MoveXYHeading to put finish boolean in the SmartDashboard. */
  public MoveXYHeading() {
    SmartDashboard.putBoolean("MoveXYHeading", finish);
  }

  /** Initializes the command by resetting the last timestamp. */
  @Override
  public void initialize() {
    lastTimestamp = Timer.getFPGATimestamp();
    defaultCommand = drive.getDefaultCommand();
    finish = false;
    xController.setSetpoint(targetXMeters + drive.getPose().getX());
    yController.setSetpoint(targetYMeters + drive.getPose().getY());
    SmartDashboard.putBoolean("MoveXYHeading", !finish);
  }

  /**
   * Executes the command, calculating the necessary velocities using PID controllers and applying
   * them to the drive subsystem.
   */
  @Override
  public void execute() {
    double temp = Timer.getFPGATimestamp() - lastTimestamp;
    if (!xController.atSetpoint() && !yController.atSetpoint()) {
      xVelocity = xController.calculate(drive.getPose().getX());
      yVelocity = yController.calculate(drive.getPose().getY());
      angVelocity = headingController.calculate(targetHeadingDegrees);

      ChassisSpeeds speed = new ChassisSpeeds(xVelocity, yVelocity, Math.toRadians(angVelocity));

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speed, drive.getRotation()));
    } else {
      drive.runVelocity(new ChassisSpeeds());
      finish = true;
    }

    finish = temp >= TimeK.TIME_OUT;
    SmartDashboard.putBoolean("MoveXYHeading", !finish);
  }

  /**
   * Ends the command, stopping the drive subsystem.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    drive.setDefaultCommand(defaultCommand);
    drive.stopWithX();
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
