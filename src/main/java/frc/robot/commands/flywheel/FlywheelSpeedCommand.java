package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;

public class FlywheelSpeedCommand {

  private FlywheelSpeedCommand() {}

  public static Command JoystickSpeed(Flywheel flywheel, double ControllerInput) {
    return Commands.run(
        () -> {
          flywheel.runVelocity(ControllerInput * 1500);
        },
        flywheel);
  }
}
