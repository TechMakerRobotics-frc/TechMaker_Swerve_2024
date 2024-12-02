package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class OnUpdatePoseCommand extends InstantCommand {

  public OnUpdatePoseCommand(PerpetualPoseCommand poseCommand) {
    super(() -> poseCommand.setShouldUpdatePose(true));
  }
}
