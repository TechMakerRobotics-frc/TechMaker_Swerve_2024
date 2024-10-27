// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Flywheel.FlywheelCommand;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.util.PhotonVision.PhotonPose;
import frc.robot.util.PhotonVision.PhotonSim;
import frc.robot.util.RegisterAlign;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final FlywheelCommand flywheelCommand;
  // private final Intake intake;

  // Usar isso caso necessário o uso do TunningPID, basta criar um parâmetro TunningPID na classe
  // que deverá
  // receber os valores pid atualizados e, no momento da sua instanciação aqui, deverá passar "pid"
  // como argumento.
  // private final TunningPID pid = TunningPID.getInstance();

  // private final Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardNumber flywheelSpeedInside =
      new LoggedDashboardNumber("Flywheel Speed Inside", 300.0);
  private final LoggedDashboardNumber flywheelSpeedOutside =
      new LoggedDashboardNumber("Flywheel Speed Outside", 1500.0);
  private final LoggedDashboardNumber lockwheelSpeedInside =
      new LoggedDashboardNumber("Flywheel Speed Inside", 1500.0);
  private final LoggedDashboardNumber lockwheelSpeedOutside =
      new LoggedDashboardNumber("Flywheel Speed Outside", 3000.0);

  /*private final LoggedDashboardNumber intakeSpeedInput =
  new LoggedDashboardNumber("Intake Speed", 1500.0);*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new PhotonPose();
    flywheelCommand = new FlywheelCommand();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkAndTalon(0),
                new ModuleIOSparkAndTalon(1),
                new ModuleIOSparkAndTalon(2),
                new ModuleIOSparkAndTalon(3));
        flywheel = new Flywheel(new FlywheelIOVictorSPX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    // Set up auto routines
    /*NamedCommands.registerCommand(
    "Run Intake",
    Commands.startEnd(() -> intake.runVolts(intakeSpeedInput.get()), intake::stop, intake)
        .withTimeout(5.0));*/

    new RegisterAlign(30, drive);

    // Configure the button bindings
    configureButtonBindings();
    if (Constants.currentMode == Mode.SIM) {
      new PhotonSim(drive);
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    /*controller.y().onTrue(new OutsideFlywheel()).onFalse(new StopFlywheel());

    controller.a().onTrue(new InsideFlywheel()).onFalse(new StopFlywheel());

    controller.x().onTrue(new OutsideLockWheel()).onFalse(new StopLockWheel());

    controller.b().onTrue(new InsideLockWheel()).onFalse(new StopLockWheel());*/

    controller
        .y()
        .onTrue(
            flywheelCommand.runOutsideFlywheel(
                flywheelSpeedOutside.get(), lockwheelSpeedOutside.get()))
        .onFalse(flywheelCommand.stopFlywheels());
    controller
        .a()
        .onTrue(flywheelCommand.runInsideFlywheel(flywheelSpeedInside.get()))
        .onFalse(flywheelCommand.stopFlywheels());
    controller
        .x()
        .onTrue(flywheelCommand.runOutsideLockWheel(lockwheelSpeedOutside.get()))
        .onFalse(flywheelCommand.stopLockWheel());
    controller
        .b()
        .onTrue(flywheelCommand.runInsideLockWheel(lockwheelSpeedInside.get()))
        .onFalse(flywheelCommand.stopLockWheel());

    controller.rightBumper().whileTrue(new AlignCommand(4, 20000, drive));

    /*controller
    .y()
    .onTrue(new InstantCommand(() -> intake.runVelocity(intakeSpeedInput.get()), intake))
    .onFalse(new InstantCommand(intake::stop, intake));*/

    controller.rightBumper().whileTrue(new AlignCommand(4, 20000, drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("AutoTest1");
    // return new AlignCommand(2000, drive);
  }
}
