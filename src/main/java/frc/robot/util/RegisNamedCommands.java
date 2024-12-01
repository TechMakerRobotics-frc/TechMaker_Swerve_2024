package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.flywheel.FlywheelDistanceCommand;
import frc.robot.commands.flywheel.InsideFlywheelCommand;
import frc.robot.commands.flywheel.OutsideFlywheelCommand;
import frc.robot.commands.flywheel.StopFlywheelCommand;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.InsideIntakeCommand;
import frc.robot.commands.intake.OutsideIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.commands.lockwheel.AlignBall;
import frc.robot.commands.lockwheel.InsideLockwheelCommand;
import frc.robot.commands.lockwheel.OutsideLockwheelCommand;
import frc.robot.commands.lockwheel.StopLockwheelCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lockwheel.Lockwheel;
import org.photonvision.PhotonCamera;

public class RegisNamedCommands {

  private Flywheel flywheel;
  private Intake intake;
  private Lockwheel lockwheel;

  /** Register commands in the pathplanner. */
  public RegisNamedCommands(
      Flywheel flywheel, PhotonCamera camera, Intake intake, Lockwheel lockwheel) {
    this.flywheel = flywheel;
    this.intake = intake;
    this.lockwheel = lockwheel;

    RegisterFlywheel(camera);
    RegisterIntake();
    RegisterLockwheel();
  }

  private void RegisterFlywheel(PhotonCamera camera) {
    NamedCommands.registerCommand("Outside Flywheel", new OutsideFlywheelCommand(flywheel, 1000));
    NamedCommands.registerCommand("Inside Flywheel", new InsideFlywheelCommand(flywheel, 1000));
    NamedCommands.registerCommand("Stop Flywheels", new StopFlywheelCommand(flywheel));
    NamedCommands.registerCommand(
        "Flywheel Distance", new FlywheelDistanceCommand(camera, flywheel));
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
