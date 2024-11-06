package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.Vision.VisionPose;
import frc.robot.Vision.VisionSim;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FlywheelCommand;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RegisterAlign;

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

  // Usar isso caso necessário o uso do TunningPID, basta criar um parâmetro TunningPID na classe
  // que deverá
  // receber os valores pid atualizados e, no momento da sua instanciação aqui, deverá passar "pid"
  // como argumento.
  // private final TunningPID pid = TunningPID.getInstance();

  // private final Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedTunableNumber flywheelSpeedInside =
      new LoggedTunableNumber("Flywheel Speed Inside", 300.0);
  private final LoggedTunableNumber flywheelSpeedOutside =
      new LoggedTunableNumber("Flywheel Speed Outside", 3000.0);
  private final LoggedTunableNumber lockwheelSpeedInside =
      new LoggedTunableNumber("Lockwheel Speed Inside", 3000.0);
  private final LoggedTunableNumber lockwheelSpeedOutside =
      new LoggedTunableNumber("Lockwheel Speed Outside", 3000.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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

    new RegisterAlign(30, drive);
    VisionPose pose = new VisionPose();

    // Configure the button bindings
    configureButtonBindings();
    if (Constants.currentMode == Mode.SIM) {
      new VisionSim(drive);
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

    controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

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
    controller.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));
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
