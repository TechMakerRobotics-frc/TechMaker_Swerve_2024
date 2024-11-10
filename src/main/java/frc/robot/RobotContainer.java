package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FlywheelCommand;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RegisterAlign;
import frc.robot.vision.VisionPose;
import frc.robot.vision.VisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Flywheel flywheel;
  private final FlywheelCommand flywheelCommand;
  public VisionPose pose;

  // Usar isso caso necessário o uso do TunningPID, basta criar um parâmetro TunningPID na classe
  // que deverá
  // receber os valores pid atualizados e, no momento da sua instanciação aqui, deverá passar "pid"
  // como argumento.
  // private final TunningPID pid = TunningPID.getInstance();

  // private final Intake intake;

  // Driver Controller
  private final CommandXboxController DriverController = new CommandXboxController(0);

  // Operator Controller
  private final CommandXboxController OperatorController = new CommandXboxController(1);

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
    pose = new VisionPose(drive);

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
    // driver commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> DriverController.getLeftY(),
            () -> DriverController.getLeftX(),
            () -> -DriverController.getRightX()));

    DriverController.povRight()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    DriverController.rightBumper().whileTrue(new AlignCommand(4, 20000, drive));
    DriverController.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));

    DriverController.povUp().onTrue(new InstantCommand(() -> VisionPose.updateOdometryPose(true)));
    DriverController.povDown()
        .onTrue(new InstantCommand(() -> VisionPose.updateOdometryPose(false)));

    // Operator commands
    OperatorController.y()
        .onTrue(
            flywheelCommand.runOutsideFlywheel(
                flywheelSpeedOutside.get(), lockwheelSpeedOutside.get()))
        .onFalse(flywheelCommand.stopFlywheels());
    OperatorController.a()
        .onTrue(flywheelCommand.runInsideFlywheel(flywheelSpeedInside.get()))
        .onFalse(flywheelCommand.stopFlywheels());
    OperatorController.x()
        .onTrue(flywheelCommand.runOutsideLockWheel(lockwheelSpeedOutside.get()))
        .onFalse(flywheelCommand.stopLockWheel());
    OperatorController.b()
        .onTrue(flywheelCommand.runInsideLockWheel(lockwheelSpeedInside.get()))
        .onFalse(flywheelCommand.stopLockWheel());
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
