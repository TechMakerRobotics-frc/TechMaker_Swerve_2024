// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated intake mechanism. */
public class IntakeSim extends LinearSystemSim<N1, N1, N1> {
  // Gearbox for the intake.
  private final DCMotor m_gearbox;

  // The gearing from the motors to the output.
  private final double m_gearing;

  /**
   * Creates a simulated intake mechanism.
   *
   * @param plant The linear system that represents the intake. This system can be created with
   *     {@link edu.wpi.first.math.system.plant.LinearSystemId#createIntakeSystem(DCMotor, double,
   *     double)}.
   * @param gearbox The type of and number of motors in the intake gearbox.
   * @param gearing The gearing of the intake (numbers greater than 1 represent reductions).
   */
  public IntakeSim(LinearSystem<N1, N1, N1> plant, DCMotor gearbox, double gearing) {
    super(plant);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated intake mechanism.
   *
   * @param plant The linear system that represents the intake.
   * @param gearbox The type of and number of motors in the intake gearbox.
   * @param gearing The gearing of the intake (numbers greater than 1 represent reductions).
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public IntakeSim(
      LinearSystem<N1, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      Matrix<N1, N1> measurementStdDevs) {
    super(plant, measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated intake mechanism.
   *
   * @param gearbox The type of and number of motors in the intake gearbox.
   * @param gearing The gearing of the intake (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the intake. If this is unknown, use the
   *     {@link #IntakeSim(LinearSystem, DCMotor, double, Matrix)} constructor.
   */
  public IntakeSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
    super(LinearSystemId.createFlywheelSystem(gearbox, jKgMetersSquared, gearing));
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Creates a simulated intake mechanism.
   *
   * @param gearbox The type of and number of motors in the intake gearbox.
   * @param gearing The gearing of the intake (numbers greater than 1 represent reductions).
   * @param jKgMetersSquared The moment of inertia of the intake. If this is unknown, use the
   *     {@link #IntakeSim(LinearSystem, DCMotor, double, Matrix)} constructor.
   * @param measurementStdDevs The standard deviations of the measurements.
   */
  public IntakeSim(
      DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N1, N1> measurementStdDevs) {
    super(
        LinearSystemId.createFlywheelSystem(gearbox, jKgMetersSquared, gearing),
        measurementStdDevs);
    m_gearbox = gearbox;
    m_gearing = gearing;
  }

  /**
   * Sets the intake's state.
   *
   * @param velocityRadPerSec The new velocity in radians per second.
   */
  public void setState(double velocityRadPerSec) {
    setState(VecBuilder.fill(velocityRadPerSec));
  }

  /**
   * Returns the intake velocity.
   *
   * @return The intake velocity.
   */
  public double getAngularVelocityRadPerSec() {
    return getOutput(0);
  }

  /**
   * Returns the intake velocity in RPM.
   *
   * @return The intake velocity in RPM.
   */
  public double getAngularVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getOutput(0));
  }

  /**
   * Returns the intake current draw.
   *
   * @return The intake current draw.
   */
  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is spinning
    // 2x faster than the intake
    return m_gearbox.getCurrent(getAngularVelocityRadPerSec() * m_gearing, m_u.get(0, 0))
        * Math.signum(m_u.get(0, 0));
  }

  /**
   * Sets the input voltage for the intake.
   *
   * @param volts The input voltage.
   */
  public void setInputVoltage(double volts) {
    setInput(volts);
  }
}
