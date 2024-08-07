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
  private static PIDController vXSpeakerController =
      new PIDController(
          AlignConstants.VX_SPEAKER_P, AlignConstants.VX_SPEAKER_I, AlignConstants.VX_SPEAKER_D);
  private static PIDController vOmegaSpeakerController =
      new PIDController(
          AlignConstants.VOMEGA_SPEAKER_P,
          AlignConstants.VOMEGA_SPEAKER_I,
          AlignConstants.VOMEGA_SPEAKER_D);

  private final Timer timer = new Timer();
  private double _timeout, vx, vy, omega;
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
    _timeout = timeout;
  }

  @Override
  public void initialize() {
    vOmegaSpeakerController.setSetpoint(180);
    vXSpeakerController.setSetpoint(0);
    vYSpeakerController.setSetpoint(2);
    timer.reset();
    timer.start();
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = PhotonTags.getLatestPipeline();
    PhotonTrackedTarget t = PhotonTags.getBestTarget(p);
    if (PhotonTags.hasUsedPipeline(3)) {
      printToDashboard();

      vx = vXSpeakerController.calculate((PhotonTags.getYaw(t))) * -1;
      vy = vYSpeakerController.calculate(PhotonTags.getBestCamera(t).getX()) * -1;

      omega =
          vOmegaSpeakerController.calculate(
              Math.abs(PhotonTags.getBestCamera(t).getRotation().toRotation2d().getDegrees()));
      omega =
          Math.copySign(
                  omega, PhotonTags.getBestCamera(t).getRotation().toRotation2d().getDegrees())
              * -1;

      drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, drive.getRotation()));
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
    SmartDashboard.putNumber("Current Tag", PhotonTags.getTargetId(t));
    SmartDashboard.putNumber("omega", omega);
    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("vy", vy);
  }
}
