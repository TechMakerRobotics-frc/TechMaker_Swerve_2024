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
  private static PIDController vYController =
      new PIDController(
          AlignConstants.VY_SPEAKER_P, AlignConstants.VY_SPEAKER_I, AlignConstants.VY_SPEAKER_D);
  private static PIDController vXController =
      new PIDController(
          AlignConstants.VX_SPEAKER_P, AlignConstants.VX_SPEAKER_I, AlignConstants.VX_SPEAKER_D);
  private static PIDController vOmegaController =
      new PIDController(
          AlignConstants.V_OMEGA_SPEAKER_P,
          AlignConstants.V_OMEGA_SPEAKER_I,
          AlignConstants.V_OMEGA_SPEAKER_D);

  private final Timer timer = new Timer();
  private double timeout, vx, vy, omega;
  private int usedTag;
  private Command defaultCommand;
  private Drive drive;

  private boolean isFinished = false;

  /**
   * Constructs a new AlignCommand.
   *
   * @param timeout the time in seconds before the command times out
   * @param drive the Drive subsystem used by this command
   * @param usedTag the used apriltag to align
   */
  public AlignCommand(int usedTag, double timeout, Drive drive) {
    this.drive = drive;
    this.timeout = timeout;
    this.usedTag = usedTag;
  }

  @Override
  public void initialize() {
    vOmegaController.setSetpoint(180);
    vXController.setSetpoint(0);
    vYController.setSetpoint(2);
    timer.reset();
    timer.start();
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = PhotonTags.getLatestPipeline();
    if (PhotonTags.hasTarget(p)) {
      PhotonTrackedTarget t = PhotonTags.getBestTarget(p);
      printToDashboard();

      vx = vXController.calculate((PhotonTags.getYaw(t))) * -1;
      vy = vYController.calculate(PhotonTags.getBestCamera(t).getX()) * -1;
      omega = vOmegaController.calculate(Math.abs(PhotonTags.getAngle(t)));
      omega = Math.copySign(omega, PhotonTags.getAngle(t)) * -1;

      drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, drive.getRotation()));
    }
    isFinished = timer.get() >= timeout;
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
    SmartDashboard.putNumber("Target Angle", PhotonTags.getAngle(t));
    SmartDashboard.putNumber("Target Distance", PhotonTags.getDistance(t));
    SmartDashboard.putNumber(
        "Target Distance with hypotenuse calc", PhotonTags.getDistanceHypotenuse());
  }
}
