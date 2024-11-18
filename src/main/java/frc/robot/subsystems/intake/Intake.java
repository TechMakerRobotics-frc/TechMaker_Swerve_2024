package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  private final DigitalInput insideSensor = new DigitalInput(0);
  private final DigitalInput outsideSensor = new DigitalInput(1);

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log intake setpoint
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  /** Stops the intake. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  /**
   * Returns true if the sensor has an object.
   *
   * @return get sensor
   */
  public boolean insideSensorIsTrue() {
    return insideSensor.get();
  }

  /**
   * Returns true if the sensor has an object.
   *
   * @return get sensor
   */
  public boolean outsideSensorIsTrue() {
    return outsideSensor.get();
  }
}
