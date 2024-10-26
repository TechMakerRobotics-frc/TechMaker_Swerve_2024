package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOVictorSPX;
import frc.robot.subsystems.flywheel.LockWheel;

public class OutsideFlywheel extends InstantCommand {

  private Flywheel flywheel;
  private Flywheel lockWheel;
  private Timer timer;

  @Override
  public void initialize() {
    flywheel = new Flywheel(new FlywheelIOVictorSPX());
    lockWheel = new Flywheel(new LockWheel());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    flywheel.runVelocity(5000);
    if (timer.get() > 5000) {
      lockWheel.runVelocity(1000);
    }
  }
}
