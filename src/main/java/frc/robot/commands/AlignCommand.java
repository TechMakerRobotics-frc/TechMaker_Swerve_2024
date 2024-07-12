// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.IdTargetHeight;
import frc.robot.util.PhotonVision.PhotonTags;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.*;

public class AlignCommand extends Command {
  /*private static PIDController vyAmpController =
    new PIDController(AlignConstants.kvyAmpP, AlignConstants.kvyAmpI, AlignConstants.kvyAmpD);
*/
  PIDController forwardController = new PIDController(AlignConstants.LINEAR_P, 0, AlignConstants.LINEAR_D);
  PIDController turnController = new PIDController(AlignConstants.ANGULAR_P, 0, AlignConstants.ANGULAR_D);
  
  private Drive drive;
  private final Timer timer = new Timer();
  private double _timeout, tagInMeters, goal, forwardSpeed, rotationSpeed;
  private Command defaultCommand;
  private int tag;

  private boolean isFinished = false;

  public AlignCommand(double timeout,int tag, double goal, Drive drive) {
    IdTargetHeight idToHeight = new IdTargetHeight(tag);
    this.tagInMeters = idToHeight.getTagMeters();
    this.drive = drive;
    this.tag = tag;
    this.goal = goal;
    //vyAmpController.setSetpoint(AlignConstants.kTargetArea);
    _timeout = timeout;
  }

  @Override
  public void initialize() {
    //vyAmpController.reset();
    timer.reset();
    timer.start();
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
  }

  @Override
  public void execute() {
    PhotonPipelineResult p = PhotonTags.getLatestPipeline();

    if(PhotonTags.hasTarget(p)){    
      double range =
        PhotonUtils.calculateDistanceToTargetMeters(
                AlignConstants.CAMERA_HEIGHT_METERS,
                tagInMeters,
                AlignConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(PhotonTags.getBestTarget(p).getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -forwardController.calculate(range, goal);
      rotationSpeed = -turnController.calculate(PhotonTags.getBestTarget(p).getYaw(), 0);
    }

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, rotationSpeed, 0, drive.getRotation()));
    /* 
    if (PhotonTags.hasTarget(p)) {
      PhotonTrackedTarget t = PhotonTags.getBestTarget(p);
      SmartDashboard.putData("PID AMP", vyAmpController);
      double vo = PhotonTags.getYaw(t) / 20;
      double vy = vyAmpController.calculate(PhotonTags.getArea(t));
      SmartDashboard.putNumber("Angular", vo);
      SmartDashboard.putNumber("X", vy);
      SmartDashboard.putNumber("Distance", PhotonTags.getArea(t));
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vo, vy, 0, drive.getRotation()));
    */

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
