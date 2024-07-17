package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PhotonVision.PhotonTags;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Command to align the robot using vision targets detected by PhotonVision. */
public class AlignCommand extends Command {
  private static PIDController vYSpeakerController =
      new PIDController(
          AlignConstants.VY_SPEAKER_P, AlignConstants.VY_SPEAKER_I, AlignConstants.VY_SPEAKER_D);
  private static PIDController omegaControler = new PIDController(0.75, 0, 0);
  private final Timer timer = new Timer();
  private double _timeout;
  private Command defaultCommand;
  private Drive drive;

  private boolean isFinished = false;

  /**
   * Constructs a new AlignCommand.
   *
   * @param timeout the time in seconds before the command times out
   * @param drive the Drive subsystem used by this command
   */
  public AlignCommand(double timeout, Drive drive) {
    this.drive = drive;
    vYSpeakerController.setSetpoint(AlignConstants.kTargetArea);
    _timeout = timeout;
  }

  @Override
  public void initialize() {
    vYSpeakerController.reset();
    timer.reset();
    timer.start();
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = PhotonTags.getLatestPipeline();

    if (PhotonTags.hasTarget(p)) {
      printToDashboard();
      PhotonTrackedTarget t = PhotonTags.getBestTarget(p);

      double omega = PhotonTags.getYaw(t) / 20;
      //double vy = vYSpeakerController.calculate(PhotonTags.getArea(t));
      double vx = omegaControler.calculate(PhotonTags.getDistance());

      SmartDashboard.putNumber("X", vx);
      //SmartDashboard.putNumber("Y", vy);
      SmartDashboard.putNumber("Área", PhotonTags.getArea(t));
      SmartDashboard.putData("PID ", vYSpeakerController);

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, 0, omega, drive.getRotation()));
    }

    isFinished = timer.get() >= _timeout;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    drive.setDefaultCommand(defaultCommand);
  }

  /** Prints vision targeting information to the SmartDashboard. */
  public void printToDashboard() {
    PhotonPipelineResult p = PhotonTags.getLatestPipeline();
    PhotonTrackedTarget t = PhotonTags.getBestTarget(p);
    SmartDashboard.putNumber("Tag Yaw", PhotonTags.getYaw(t));
    SmartDashboard.putNumber("Tag Pitch", PhotonTags.getPitch(t));
    SmartDashboard.putNumber("Tag Skew", PhotonTags.getSkew(t));
    SmartDashboard.putNumber("Current Tag", PhotonTags.getTargetId(t));
  }
}
