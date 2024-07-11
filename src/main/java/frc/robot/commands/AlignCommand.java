// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PhotonTags;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignCommand extends Command {
  private static PIDController vyAmpController =
      new PIDController(AlignConstants.kvyAmpP, AlignConstants.kvyAmpI, AlignConstants.kvyAmpD);
  private Drive drive;
  private final Timer timer = new Timer();
  private double _timeout;
  private Command defaultCommand;

  private boolean isFinished = false;
  public AlignCommand(double timeout, Drive drive) {
    this.drive = drive;
    vyAmpController.setSetpoint(AlignConstants.kTargetArea);
    _timeout = timeout;
  }

  @Override
  public void initialize() {
    vyAmpController.reset();
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
      SmartDashboard.putData("PID AMP", vyAmpController);
      double vo = PhotonTags.getYaw(t) / 20;
      double vy = vyAmpController.calculate(PhotonTags.getArea(t));
      SmartDashboard.putNumber("Angular", vo);
      SmartDashboard.putNumber("X", vy);
      SmartDashboard.putNumber("Distance", PhotonTags.getArea(t));
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vo, vy, 0, drive.getRotation()));

      if (vyAmpController.atSetpoint()) {}
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
