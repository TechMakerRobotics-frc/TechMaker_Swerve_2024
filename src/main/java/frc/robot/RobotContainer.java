package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.*;
import frc.robot.subsystems.intake.*;
import frc.robot.util.*;
import frc.robot.vision.*;
import frc.robot.vision.VisionConstants.CameraConstants;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final FlywheelCommand flywheelCommand;
  private final IntakeCommand intakeCommand;

  private final Flywheel flywheel;
  private final Intake intake;

  public final VisionPose pose;

  private final PhotonCamera flCam = new PhotonCamera(CameraConstants.CAMERA_FL_NAME);
  private final PhotonCamera frCam = new PhotonCamera(CameraConstants.CAMERA_FR_NAME);
  private final PhotonCamera limelight = new PhotonCamera(CameraConstants.LIMELIGHT_NAME);

  public final LedsControl leds;

  public VisionSim visionSim;

  private int currentLedState = 0; // Índice inicial para o estado do LED

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
  private LoggedTunableNumber flywheelSpeedInside =
      new LoggedTunableNumber("Flywheel Speed Inside", 300.0);
  private LoggedTunableNumber flywheelSpeedOutside =
      new LoggedTunableNumber("Flywheel Speed Outside", 3000.0);
  private LoggedTunableNumber lockwheelSpeedInside =
      new LoggedTunableNumber("Lockwheel Speed Inside", 3000.0);
  private LoggedTunableNumber lockwheelSpeedOutside =
      new LoggedTunableNumber("Lockwheel Speed Outside", 3000.0);
  /*private LoggedTunableNumber flywheelSpeedFly =
  new LoggedTunableNumber("Flywheel Speed Fly", 2000);*/

  private LoggedTunableNumber intakeSpeedInside =
      new LoggedTunableNumber("Intake Speed Inside", 200);
  private LoggedTunableNumber intakeSpeedOutside =
      new LoggedTunableNumber("Intake Speed Outside", 200);

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
        new RegisNamedCommands(flywheel, intake);
        flywheelCommand = new FlywheelCommand(flywheel);
        intakeCommand = new IntakeCommand(intake);
        pose = new VisionPose(drive, flCam, frCam, limelight);
        leds = new LedsControl();
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
        visionSim = new VisionSim(drive, flCam, frCam, limelight);
        flywheelCommand = new FlywheelCommand(flywheel);
        intakeCommand = new IntakeCommand(intake);
        pose = new VisionPose(drive, flCam, frCam, limelight);
        leds = new LedsControl();
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
        flywheelCommand = new FlywheelCommand(flywheel);
        intakeCommand = new IntakeCommand(intake);
        pose = new VisionPose(drive, flCam, frCam, limelight);
        leds = new LedsControl();
        break;
    }

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

    DriverController.povRight()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    DriverController.rightBumper().whileTrue(new AlignCommand(drive, limelight, 4, 20000));
    DriverController.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));

    DriverController.povUp().onTrue(new InstantCommand(() -> VisionPose.updateOdometryPose(true)));
    DriverController.povDown()
        .onTrue(new InstantCommand(() -> VisionPose.updateOdometryPose(false)));

    // DriverController.a().whileTrue(new TrajectoryCommand(drive));

    // Operator commands

    // flywheel
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

    OperatorController.rightBumper()
        .whileTrue(new FlywheelDistanceCommand(limelight, flywheel))
        .onFalse(flywheelCommand.stopFlywheels());

    // OperatorController.leftBumper().onTrue(flywheelCommand.runFlywheel(flywheelSpeedFly.get()));

    // OperatorController.rightBumper().onTrue(flywheelCommand.stopFlywheels());

    // intake
    OperatorController.povUp()
        .onTrue(intakeCommand.runInsideIntake(intakeSpeedInside.get()))
        .onFalse(intakeCommand.stopIntake());

    OperatorController.povDown()
        .onTrue(intakeCommand.runOutsideIntake(intakeSpeedOutside.get()))
        .onFalse(intakeCommand.stopIntake());

    OperatorController.povLeft().onTrue(new InstantCommand(() -> intake.extend()));
    OperatorController.povLeft().onTrue(new InstantCommand(() -> intake.retract()));

    // leds
    OperatorController.leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  // Obtenha a lista de estados disponíveis
                  Constants.LedState[] states = Constants.LedState.values();

                  // Atualiza o estado atual
                  currentLedState = (currentLedState + 1) % states.length;

                  // Altera para o próximo estado
                  leds.setState(states[currentLedState]);

                  // Mensagem de debug (opcional)
                  SmartDashboard.putString(
                      "Led state", "LED alterado para: " + states[currentLedState]);
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("AutoTest1");
  }
}
