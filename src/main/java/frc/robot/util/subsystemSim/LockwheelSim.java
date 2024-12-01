// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.subsystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated Lockwheel mechanism. */
public class LockwheelSim extends LinearSystemSim<N1, N1, N1> {
  // Gearbox for the Lockwheel.
  private final DCMotor m_gearbox;

  // The gearing from the motors to the output.
  private final double m_gearing;

  /**
   * Creates a simulated Lockwheel mechanism.
   *
   * @param plant The linear system that represents the Lockwheel. This system can be created with
   *     {@link edu.wpi.first.math.system.plant.LinearSystemId#createLockwheelSystem(DCMotor,
   *     double, double)}.
   * @param gearbox The type of and number of motors in the Lockwheel gearbox.
   * @param gearing The gearing of the Lockwheel (numbers greater than 1 represent reductions).
   */
  public LockwheelSim(LinearSystem<N1, N1, N1> plant, DCMotor gearbox, double gearing) {
    super(plant);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated Lockwheel mechanism.
   *
   * @param plant The linear system that represents the Lockwheel.
   * @param gearbox The type of and number of motors in the Lockwheel gearbox.
   * @param gearing The gearing of the Lockwheel (numbers greater than 1 represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public LockwheelSim(
      LinearSystem<N1, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      Matrix<N1, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated Lockwheel mechanism.
   *
   * @param gearbox The type of and number of motors in the Lockwheel gearbox.
   * @param gearing The gearing of the Lockwheel (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the Lockwheel. If this is unknown, use the
   *     {@link #LockwheelSim(LinearSystem, DCMotor, double, Matrix)} constructor.
   */
  public LockwheelSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
    super(LinearSystemId.createFlywheelSystem(gearbox, jKgMetersSquared, gearing));
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated Lockwheel mechanism.
   *
   * @param gearbox The type of and number of motors in the Lockwheel gearbox.
   * @param gearing The gearing of the Lockwheel (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the Lockwheel. If this is unknown, use the
   *     {@link #LockwheelSim(LinearSystem, DCMotor, double, Matrix)} constructor.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public LockwheelSim(
      DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N1, N1> measurementStdDevs) {
    super(
        LinearSystemId.createFlywheelSystem(gearbox, jKgMetersSquared, gearing),
        measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Sets the Lockwheel's state.
   *
   * @param velocityRadPerSec The new velocity in radians per second.
   */
  public void setState(double velocityRadPerSec) {
    setState(VecBuilder.fill(velocityRadPerSec));
  }

  /**
   * Returns the Lockwheel velocity.
   *
   * @return The Lockwheel velocity.
   */
  public double getAngularVelocityRadPerSec() {
    return getOutput(0);
  }

  /**
   * Returns the Lockwheel velocity in RPM.
   *
   * @return The Lockwheel velocity in RPM.
   */
  public double getAngularVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getOutput(0));
  }

  /**
   * Returns the Lockwheel current draw.
   *
   * @return The Lockwheel current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is spinning
    // 2x faster than the Lockwheel
    return m_gearbox.getCurrent(getAngularVelocityRadPerSec() * m_gearing, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the Lockwheel.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }
}
