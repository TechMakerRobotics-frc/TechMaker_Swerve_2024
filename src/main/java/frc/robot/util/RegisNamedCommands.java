package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.FlywheelCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

public class RegisNamedCommands {

  FlywheelCommand flywheel;
  IntakeCommand intake;

  /** Register commands in the pathplanner. */
  public RegisNamedCommands(Flywheel flywheel, Intake intake) {
    this.flywheel = new FlywheelCommand(flywheel);
    this.intake = new IntakeCommand(intake);

    RegisterFlywheel();
    RegisterIntake();
  }

  private void RegisterFlywheel() {
    NamedCommands.registerCommand("Outside Flywheel", flywheel.runOutsideFlywheel(3000, 3000));
    NamedCommands.registerCommand("Inside Flywheel", flywheel.runInsideFlywheel(1500));
    NamedCommands.registerCommand("Inside LockWheel", flywheel.runInsideLockWheel(1000));
    NamedCommands.registerCommand("Outside LockWheel", flywheel.runOutsideLockWheel(1000));
    NamedCommands.registerCommand("Stop Flywheels", flywheel.stopFlywheels());
    NamedCommands.registerCommand("Stop LockWheel", flywheel.stopLockWheel());
  }

  private void RegisterIntake() {
    NamedCommands.registerCommand("Inside Intake", intake.runInsideIntake(2000));
    NamedCommands.registerCommand("Outside Intake", intake.runOutsideIntake(2000));
    NamedCommands.registerCommand("Stop Intake", intake.stopIntake());
  }
}
