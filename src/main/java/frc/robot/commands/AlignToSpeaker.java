// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.AlignConstants;
import frc.robot.commands.CommandConstants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;

public class AlignToSpeaker extends Command {
  private static PIDController vyStageController =
      new PIDController(
          AlignConstants.kvyStageP, AlignConstants.kvyStageI, AlignConstants.kvyStageD);
  private final Timer timer = new Timer();
  private Drive drive;
  private double _timeout;
  private Command defaultCommand;
  private LimelightSubsystem limelight = LimelightSubsystem.getInstance();

  public AlignToSpeaker(double timeout, Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    vyStageController.setSetpoint(AlignConstants.kDistanceFromSpeakerToShoot);
    vyStageController.setTolerance(0.3);
    _timeout = timeout;
  }

  @Override
  public void initialize() {
    limelight.startLimelight();
    vyStageController.reset();
    timer.reset();
    timer.start();
    defaultCommand = drive.getDefaultCommand();
    drive.removeDefaultCommand();
  }

  @Override
  public void execute() {
    if (limelight.atSpeaker()) {
      double vo = -limelight.getTx() / 50;
      double vy = vyStageController.calculate(limelight.getDistance());
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(-vy, 0, vo, drive.getRotation()));
    }
  }

  @Override
  public boolean isFinished() {
    return (timer.get() >= _timeout);
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setPipeline(LimelightConstants.kPosePipeline);
    drive.stopWithX();
    drive.setDefaultCommand(defaultCommand);
  }
}
