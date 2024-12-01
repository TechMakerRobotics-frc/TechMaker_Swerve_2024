package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.flywheel.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.lockwheel.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lockwheel.Lockwheel;
import frc.robot.vision.VisionPose;

public class RegisNamedCommands {

  private Flywheel flywheel;
  private Intake intake;
  private Lockwheel lockwheel;
  private VisionPose visionPose;

  /** Register commands in the pathplanner. */
  public RegisNamedCommands(
      Flywheel flywheel, Intake intake, Lockwheel lockwheel, VisionPose visionPose) {
    this.flywheel = flywheel;
    this.intake = intake;
    this.lockwheel = lockwheel;
    this.visionPose = visionPose;

    RegisterFlywheel();
    RegisterIntake();
    RegisterLockwheel();
  }

  private void RegisterFlywheel() {
    NamedCommands.registerCommand("Outside Flywheel", new OutsideFlywheelCommand(flywheel, 1000));
    NamedCommands.registerCommand("Inside Flywheel", new InsideFlywheelCommand(flywheel, 1000));
    NamedCommands.registerCommand("Stop Flywheels", new StopFlywheelCommand(flywheel));
    NamedCommands.registerCommand(
        "Flywheel Distance", new FlywheelDistanceCommand(flywheel, visionPose));
  }

  private void RegisterIntake() {
    NamedCommands.registerCommand("Inside Intake", new InsideIntakeCommand(intake, 250));
    NamedCommands.registerCommand("Outside Intake", new OutsideIntakeCommand(intake, 300));
    NamedCommands.registerCommand("Stop Intake", new StopIntakeCommand(intake));
    NamedCommands.registerCommand("Extend Intake", new ExtendIntakeCommand(intake));
    NamedCommands.registerCommand("Retract Intake", new RetractIntakeCommand(intake));
  }

  private void RegisterLockwheel() {
    NamedCommands.registerCommand(
        "Inside Lockwheel", new InsideLockwheelCommand(lockwheel, 1000.00));
    NamedCommands.registerCommand(
        "Outside Lockwheel", new OutsideLockwheelCommand(lockwheel, 1000.00));
    NamedCommands.registerCommand("Stop Lockwheel", new StopLockwheelCommand(lockwheel));
    NamedCommands.registerCommand("Align Ball", new AlignBall(lockwheel));
  }
}
