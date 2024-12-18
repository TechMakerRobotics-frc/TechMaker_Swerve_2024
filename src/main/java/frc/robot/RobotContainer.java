package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto;
import frc.robot.commands.drive.*;
import frc.robot.commands.flywheel.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.leds.*;
import frc.robot.commands.lockwheel.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.lockwheel.*;
import frc.robot.util.*;
import frc.robot.vision.*;

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
  private final Intake intake;
  private final Lockwheel lockwheel;

  private final VisionPose visionPose;

  private final PerpetualPoseCommand poseCommand;

  public final Leds leds;

  public VisionSim visionSim;

  private int currentLedState = 0; // indice inicial para o estado do LED

  private final Command[] ledCommands;
  private Trigger ShootWithControl;
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

  // tunable flywheel velocity
  private LoggedTunableNumber flywheelSpeedInside =
      new LoggedTunableNumber("Flywheel Speed Inside", 300.0);
  private LoggedTunableNumber flywheelSpeedOutside =
      new LoggedTunableNumber("Flywheel Speed Outside", 3000.0);

  // tunable intake velocity
  private LoggedTunableNumber intakeSpeedInside =
      new LoggedTunableNumber("Intake Speed Inside", 600);
  private LoggedTunableNumber intakeSpeedOutside =
      new LoggedTunableNumber("Intake Speed Outside", 600);

  // tunable lockwheel velocity
  private LoggedTunableNumber lockwheelSpeedInside =
      new LoggedTunableNumber("Flywheel Speed Inside", 800);
  private LoggedTunableNumber lockwheelSpeedOutside =
      new LoggedTunableNumber("Flywheel Speed Outside", 800);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
        intake = new Intake(new IntakeIOSparkMax());
        lockwheel = new Lockwheel(new LockwheelIOVictorSPX());
        visionPose = new VisionPose();
        poseCommand = new PerpetualPoseCommand(drive, visionPose);
        leds = new Leds();
        new RegisNamedCommands(flywheel, intake, lockwheel, visionPose, drive, leds);

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
        intake = new Intake(new IntakeIOSim());
        lockwheel = new Lockwheel(new LockwheelIOSim());
        visionPose = new VisionPose();
        poseCommand = new PerpetualPoseCommand(drive, visionPose);
        visionSim =
            new VisionSim(
                drive,
                visionPose.getFLManager().getCamera(),
                visionPose.getFRManager().getCamera(),
                visionPose.getTargetManager().getCamera());
        leds = new Leds();
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
        intake = new Intake(new IntakeIO() {});
        lockwheel = new Lockwheel(new LockwheelIO() {});
        visionPose = new VisionPose();
        poseCommand = new PerpetualPoseCommand(drive, visionPose);
        leds = new Leds();
        break;
    }

    ShootWithControl = new Trigger(() -> (OperatorController.getRightTriggerAxis() > 0.1));

    ledCommands =
        new Command[] {
          new LedRed(leds),
          new LedGreen(leds),
          new LedBlue(leds),
          new LedOff(leds),
          new LedWhite(leds),
          new LedYellow(leds),
          new LedCian(leds),
          new LedRainbow(leds)
        };

    // Configure the button bindings
    configureButtonBindings();
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
            () -> -DriverController.getLeftY(),
            () -> -DriverController.getLeftX(),
            () -> -DriverController.getRightX()));

    ShootWithControl.whileTrue(
        new FlywheelSpeedCommand(flywheel, OperatorController.getRightTriggerAxis()));

    DriverController.povRight()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    DriverController.rightBumper().whileTrue(new AlignToTag(drive, visionPose, 20000));

    DriverController.povUp().onTrue(new OnUpdatePoseCommand(poseCommand));
    DriverController.povDown().onTrue(new OffUpdatePoseCommand(poseCommand));

    // DriverController.a().whileTrue(new TrajectoryCommand(drive));

    // Operator commands

    // flywheel
    OperatorController.y()
        .onTrue(new OutsideFlywheelCommand(flywheel, flywheelSpeedOutside.get()))
        .onFalse(new StopFlywheelCommand(flywheel));
    OperatorController.a()
        .onTrue(new InsideFlywheelCommand(flywheel, flywheelSpeedInside.get()))
        .onFalse(new StopFlywheelCommand(flywheel));

    OperatorController.rightBumper()
        .whileTrue(new FlywheelDistanceCommand(flywheel, visionPose))
        .onFalse(new StopFlywheelCommand(flywheel));

    // intake
    OperatorController.povUp()
        .onTrue(new InsideIntakeCommand(intake, intakeSpeedInside.get()))
        .onFalse(new StopIntakeCommand(intake));
    OperatorController.povDown()
        .onTrue(new OutsideIntakeCommand(intake, intakeSpeedOutside.get()))
        .onFalse(new StopIntakeCommand(intake));

    OperatorController.povLeft().onTrue(new ExtendIntakeCommand(intake));
    OperatorController.povRight().onTrue(new RetractIntakeCommand(intake));

    OperatorController.leftBumper().onTrue(new AlignBall(lockwheel));

    // lockwheel
    OperatorController.x()
        .onTrue(new OutsideLockwheelCommand(lockwheel, lockwheelSpeedInside.get()))
        .onFalse(new StopLockwheelCommand(lockwheel));
    OperatorController.b()
        .onTrue(new InsideLockwheelCommand(lockwheel, lockwheelSpeedOutside.get()))
        .onFalse(new StopLockwheelCommand(lockwheel));

    // leds
    OperatorController.leftStick()
        .onTrue(
            Commands.runOnce(
                () -> {
                  // Alterna o comando atual para o próximo na lista
                  currentLedState = (currentLedState + 1) % ledCommands.length;
                  Command nextCommand = ledCommands[currentLedState];

                  // Executa o próximo comando
                  nextCommand.schedule();
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.''
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Auto(drive, flywheel, visionPose, lockwheel, leds);
  }
}
