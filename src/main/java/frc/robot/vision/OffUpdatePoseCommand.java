package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OffUpdatePoseCommand extends InstantCommand {

  public OffUpdatePoseCommand(PerpetualPoseCommand poseCommand) {
    super(() -> poseCommand.setShouldUpdatePose(false));
  }
}
