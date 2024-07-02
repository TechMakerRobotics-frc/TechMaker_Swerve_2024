package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PhotonTags;

public class AlingCommand extends Command {

  PhotonTags vision;
  private double distanceGoal;
  private int tag;
  private Drive drive;
  private PhotonTags photonTags;

  public AlingCommand(double distanceGoal, int tag, Drive drive) {
    this.distanceGoal = distanceGoal;
    this.tag = tag;
    this.drive = drive;
    photonTags = new PhotonTags(tag, distanceGoal, drive);
  }

  @Override
  public void execute() {
    photonTags.aling();
    photonTags.go();
  }

  
  /** 
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
