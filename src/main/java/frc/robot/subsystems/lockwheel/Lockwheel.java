package frc.robot.subsystems.lockwheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.*;

public class Lockwheel extends SubsystemBase {
  private final LockwheelIO io;
  private final LockwheelIOInputsAutoLogged inputs = new LockwheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  private final DigitalInput frontSensor =
      new DigitalInput(LockwheelConstants.FRONT_SENSOR_CHANNEL);
  private final DigitalInput backSensor = new DigitalInput(LockwheelConstants.BACK_SENSOR_CHANNEL);

  /** Creates a new lockwheel. */
  public Lockwheel(LockwheelIO io) {
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
                (state) -> Logger.recordOutput("Lockwheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Lockwheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log lockwheel setpoint
    Logger.recordOutput("Lockwheel/SetpointRPM", velocityRPM);
  }

  public void rotateForward() {
    runVelocity(LockwheelConstants.VELOCITY_ROTATE_FORWARD);
  }

  public void rotateBackward() {
    runVelocity(LockwheelConstants.VELOCITY_ROTATE_BACKWARD);
  }

  /** Stops the lockwheel. */
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
  @AutoLogOutput
  public boolean frontSensorIsTrue() {
    return !frontSensor.get();
  }

  /**
   * Returns true if the sensor has an object.
   *
   * @return get sensor
   */
  @AutoLogOutput
  public boolean backSensorIsTrue() {
    return !backSensor.get();
  }
}
