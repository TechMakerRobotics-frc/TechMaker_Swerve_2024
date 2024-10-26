package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOVictorSPX;

public class InsideFlywheel extends InstantCommand {

  private Flywheel flywheel;

  @Override
  public void initialize() {
    flywheel = new Flywheel(new FlywheelIOVictorSPX());
  }

  @Override
  public void execute() {
    flywheel.runVelocity(-200);
  }
}
