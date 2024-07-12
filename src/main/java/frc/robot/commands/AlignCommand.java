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

public class AlignCommand extends Command {
  private static PIDController vYSpeakerController =
      new PIDController(
          AlignConstants.VY_SPEAKER_P, AlignConstants.VY_SPEAKER_I, AlignConstants.VY_SPEAKER_D);
  //private static PIDController omegaControler = new PIDController(0.5, 0, 0);
  private final Timer timer = new Timer();
  private double _timeout;
  private Command defaultCommand;
  private Drive drive;

  private boolean isFinished = false;

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
      PhotonTrackedTarget t = PhotonTags.getBestTarget(p);
      
      double vx = PhotonTags.getYaw(t) / 20;
      double vy = vYSpeakerController.calculate(PhotonTags.getArea(t));
      //double omega = PhotonTags.getPitch(t);

      SmartDashboard.putNumber("X", vx);
      SmartDashboard.putNumber("Y", vy);
      SmartDashboard.putNumber("Área", PhotonTags.getArea(t));
      SmartDashboard.putData("PID ", vYSpeakerController);

      drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, 0, drive.getRotation()));
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
}
