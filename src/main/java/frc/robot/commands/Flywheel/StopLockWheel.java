package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.LockWheel;

public class StopLockWheel extends InstantCommand{
    
    private Flywheel lockWheel;

    @Override
    public void initialize() {
        lockWheel = new Flywheel(new LockWheel());
    }

    @Override
    public void execute() {
        lockWheel.stop();
    }
}