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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Command to align the robot using vision targets detected by PhotonVision. */
public class AlignToBall extends Command {
  private static PIDController vXController =
      new PIDController(
          AlignConstants.VX_P, AlignConstants.VX_I, AlignConstants.VX_D);
  
  private static PIDController vOmegaController =
      new PIDController(
          AlignConstants.V_OMEGA_P,
          AlignConstants.V_OMEGA_I,
          AlignConstants.V_OMEGA_D);

  private final Timer timer = new Timer();
  private final VisionManager manager;
  private final VisionPose visionPose;
  private double timeout, vx, omega;
  private Command defaultCommand;
  private Drive drive;

  private boolean isFinished = false;

  /**
   * Constructs a new AlignBall.
   *
   * @param timeout the time in seconds before the command times out
   * @param drive the Drive subsystem used by this command
   */
  public AlignToBall(Drive drive, VisionPose visionPose, double timeout) {
    this.visionPose = visionPose;
    manager = this.visionPose.getTargetManager();
    this.drive = drive;
    this.timeout = timeout;
  }

  @Override
  public void initialize() {
    manager.getCamera().setPipelineIndex(2);
    vXController.reset();
    vXController.setSetpoint(0);
    vXController.setTolerance(0.5);
    vOmegaController.reset();
    vOmegaController.setSetpoint(180);
    vOmegaController.setTolerance(3);
    timer.reset();
    timer.start();
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = manager.getLatestPipeline();
    if (manager.hasTarget(p)) {
      PhotonTrackedTarget t = manager.getBestTarget(p);
      printToDashboard();

      vx = vXController.calculate(manager.getYaw(t)) * -1;

      omega = vOmegaController.calculate(Math.abs(manager.getAngle(t)));
      omega = Math.copySign(omega, manager.getAngle(t)) * -1;

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, omega, drive.getRotation()));
    }
    isFinished =
        timer.get() >= timeout
            || (vXController.atSetpoint()
                && vOmegaController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setDefaultCommand(defaultCommand);
    manager.getCamera().setPipelineIndex(1);
  }

  /** Prints vision targeting information to the SmartDashboard. */
  public void printToDashboard() {
    PhotonPipelineResult p = manager.getLatestPipeline();
    PhotonTrackedTarget ball = manager.getBestTarget(p);
    SmartDashboard.putNumber("Ball Yaw", manager.getYaw(ball));
    SmartDashboard.putNumber("Ball Distance", manager.getDistance(ball));
    SmartDashboard.putNumber("PID VX Error", vXController.getPositionError());
    SmartDashboard.putBoolean("PID VX At setpoint", vXController.atSetpoint());
    SmartDashboard.putNumber("PID OMEGA Error", vOmegaController.getPositionError());
    SmartDashboard.putBoolean("PID OMEGA At setpoint", vOmegaController.atSetpoint());    
  }
}
