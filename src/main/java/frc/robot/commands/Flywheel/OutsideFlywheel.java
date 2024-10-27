package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CommandConstants.FlywheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOVictorSPX;
import frc.robot.subsystems.flywheel.LockWheel;

public class OutsideFlywheel extends Command {

  private Flywheel flywheel;
  private Flywheel lockWheel;
  private Timer timer;

  public OutsideFlywheel() {
    timer = new Timer();
  }

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
    if (timer.get() > FlywheelConstants.TIME_TO_SHOOT) {
      lockWheel.runVelocity(1000);
    }
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    lockWheel.stop();
    timer.stop();
  }
}
