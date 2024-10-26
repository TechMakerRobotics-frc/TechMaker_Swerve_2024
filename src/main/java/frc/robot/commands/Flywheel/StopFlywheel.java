package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOVictorSPX;
import frc.robot.subsystems.flywheel.LockWheel;

public class StopFlywheel extends InstantCommand {

  private Flywheel flywheel;
  private Flywheel lockWheel;

  @Override
  public void initialize() {
    flywheel = new Flywheel(new FlywheelIOVictorSPX());
    lockWheel = new Flywheel(new LockWheel());
  }

  @Override
  public void execute() {
    flywheel.stop();
    lockWheel.stop();
  }
}
