package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.LockWheel;
import frc.robot.subsystems.intake.Intake;

public class AlignBall extends Command {

    private final Intake intake;
    private final Flywheel lockwheel;
    private boolean isBallAligned;

    public AlignBall(Intake intake) {
        this.intake = intake;
        lockwheel = new Flywheel(new LockWheel());

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        isBallAligned = false;
    }

    @Override
    public void execute() {
        boolean insideSensor = intake.insideSensorIsTrue();
        boolean outsideSensor = intake.outsideSensorIsTrue();

        if (insideSensor && outsideSensor) {
            isBallAligned = true;
            lockwheel.stop();
        } else if (insideSensor) {
            lockwheel.runVelocity(-1000);
        } else if (outsideSensor) {
            lockwheel.runVelocity(1000);
        } else {
            lockwheel.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return isBallAligned;
    }

    @Override
    public void end(boolean interrupted) {
        lockwheel.stop();
    }
}
