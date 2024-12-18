// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveTo extends Command {
  /** Creates a new DriveTo. */
  Drive drive;

  double x, y, rot;

  public DriveTo(double x, double y, double rot, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.rot = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setPose(new Pose2d());
    drive.runVelocity(new ChassisSpeeds(-3, 0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getPose().getX() > x;
  }
}
