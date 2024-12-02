package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.vision.VisionManager;
import frc.robot.vision.VisionPose;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Command to align the robot using vision targets detected by PhotonVision. */
public class AlignCommand extends Command {
  private static PIDController vXController =
      new PIDController(
          AlignConstants.VX_SPEAKER_P, AlignConstants.VX_SPEAKER_I, AlignConstants.VX_SPEAKER_D);
  private static PIDController vYController =
      new PIDController(
          AlignConstants.VY_SPEAKER_P, AlignConstants.VY_SPEAKER_I, AlignConstants.VY_SPEAKER_D);
  private static PIDController vOmegaController =
      new PIDController(
          AlignConstants.V_OMEGA_SPEAKER_P,
          AlignConstants.V_OMEGA_SPEAKER_I,
          AlignConstants.V_OMEGA_SPEAKER_D);

  private final Timer timer = new Timer();
  private final VisionManager manager;
  private final VisionPose visionPose;
  private double timeout, vx, vy, omega;
  // private int usedTag;
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
  public AlignCommand(Drive drive, VisionPose visionPose, double timeout) {
    this.visionPose = visionPose;
    manager = visionPose.getTargetManager();
    this.drive = drive;
    this.timeout = timeout;
  }

  @Override
  public void initialize() {
    vXController.setSetpoint(0);
    vXController.setTolerance(0.5);
    vYController.setSetpoint(2);
    vYController.setTolerance(3);
    vOmegaController.setSetpoint(180);
    vOmegaController.setTolerance(3);
    timer.reset();
    timer.start();
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
    if (manager.getCamera() != null) {
      manager.getCamera().setLED(VisionLEDMode.kOn);
    }
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = manager.getLatestPipeline();
    if (manager.hasTarget(p)) {
      PhotonTrackedTarget t = manager.getBestTarget(p);
      printToDashboard();

      vx = vXController.calculate(manager.getYaw(t)) * -1;
      vy = vYController.calculate(manager.getDistance(t));
      omega = vOmegaController.calculate(Math.abs(manager.getAngle(t)));
      omega = Math.copySign(omega, manager.getAngle(t)) * -1;

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vy, vx, omega, drive.getRotation()));
    }
    isFinished =
        timer.get() >= timeout
            || (vXController.atSetpoint()
                && vYController.atSetpoint()
                && vOmegaController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
    drive.setDefaultCommand(defaultCommand);
    manager.getCamera().setLED(VisionLEDMode.kOff);
  }

  /** Prints vision targeting information to the SmartDashboard. */
  public void printToDashboard() {
    PhotonPipelineResult p = manager.getLatestPipeline();
    PhotonTrackedTarget t = manager.getBestTarget(p);
    SmartDashboard.putNumber("Tag Yaw", manager.getYaw(t));
    SmartDashboard.putNumber("Current Tag", manager.getTargetId(t));
    SmartDashboard.putNumber("omega", omega);
    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("vy", vy);
    SmartDashboard.putNumber("Target Angle", manager.getAngle(t));
    SmartDashboard.putNumber("Target Distance", manager.getDistance(t));
    SmartDashboard.putNumber("PID VX Error", vXController.getPositionError());
    SmartDashboard.putNumber("PID VY Error", vYController.getPositionError());
    SmartDashboard.putNumber("PID OMEGA Error", vOmegaController.getPositionError());
    SmartDashboard.putBoolean("PID VX At setpoint", vXController.atSetpoint());
    SmartDashboard.putBoolean("PID VY At setpoint", vYController.atSetpoint());
    SmartDashboard.putBoolean("PID OMEGA At setpoint", vOmegaController.atSetpoint());
  }
}
