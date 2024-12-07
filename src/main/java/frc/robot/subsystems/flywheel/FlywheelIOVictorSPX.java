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

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class FlywheelIOVictorSPX implements FlywheelIO {
  private static final int ENCODER_PULSES_PER_REV = 2048; // Ajuste de acordo com o encoder usado

  private final WPI_VictorSPX leader = new WPI_VictorSPX(15);
  private final WPI_VictorSPX follower = new WPI_VictorSPX(16);

  private double currentVelocityRotations = 0;
  private double appliedVolts = 0;
  private double leaderCurrent = 0;
  private double followerCurrent = 0;

  private final Encoder encoder = new Encoder(2, 3);

  public FlywheelIOVictorSPX() {
    leader.configFactoryDefault();
    follower.configFactoryDefault();

    leader.setNeutralMode(NeutralMode.Coast);
    follower.setNeutralMode(NeutralMode.Coast);

    leader.configVoltageCompSaturation(12.0);
    leader.enableVoltageCompensation(true);
    follower.configVoltageCompSaturation(12.0);
    follower.enableVoltageCompensation(true);

    leader.configOpenloopRamp(0.1); // Slope for ramping up voltage
    encoder.reset();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Calcular a velocidade em rotações por minuto (RPM)
    currentVelocityRotations = (encoder.getRate() / ENCODER_PULSES_PER_REV) * 60.0;

    // Atualizar os inputs
    inputs.velocityRadPerSec = Units.rotationsToRadians(currentVelocityRotations);
    inputs.positionRad = Units.rotationsToRadians((double) encoder.get() / ENCODER_PULSES_PER_REV);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {leaderCurrent, followerCurrent};
  }

  @Override
  public void setVoltage(double volts) {
    leader.set(ControlMode.PercentOutput, volts / 12.0);
    follower.set(ControlMode.PercentOutput, volts / 12.0);
    appliedVolts = volts;
  }

  @Override
  public void setVoltageUpMotor(double volts) {
    leader.set(ControlMode.PercentOutput, volts / 12.0);
    appliedVolts = volts;
  }

  @Override
  public void setVoltageDownMotor(double volts) {
    follower.set(ControlMode.PercentOutput, volts / 12.0);
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    leader.set(ControlMode.PercentOutput, ffVolts / 12.0);
    follower.set(ControlMode.PercentOutput, ffVolts / 12.0);
    appliedVolts = ffVolts;
  }

  @Override
  public void stop() {
    leader.set(ControlMode.PercentOutput, 0);
    follower.set(ControlMode.PercentOutput, 0);
  }
}
