package frc.robot.subsystems.lockwheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.util.Units;

public class LockwheelIOVictorSPX implements LockwheelIO {
  private static final double GEAR_RATIO = 1.5;

  private final WPI_VictorSPX motor = new WPI_VictorSPX(17);

  private double currentVelocity = 0;
  private double appliedVolts = 0;
  private double currentAmps = 0;

  public LockwheelIOVictorSPX() {
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configVoltageCompSaturation(12.0);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(0.1); // Slope for ramping up voltage
  }

  /**
   * @param inputs
   */
  @Override
  public void updateInputs(LockwheelIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsToRadians(currentVelocity) / GEAR_RATIO;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {currentAmps};
  }

  @Override
  public void setVoltage(double volts) {
    motor.set(ControlMode.PercentOutput, volts / 12.0); // Convert to percent output
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // VictorSPX doesn't support velocity control directly without an external sensor, so we'll
    // emulate with percent output.
    double targetRPM = Units.radiansToRotations(velocityRadPerSec) * 60.0;
    currentVelocity = targetRPM / GEAR_RATIO;
    motor.set(ControlMode.PercentOutput, ffVolts / 12.0); // Open loop control
    appliedVolts = ffVolts;
  }

  @Override
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }
}
